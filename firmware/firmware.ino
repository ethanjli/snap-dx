#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>

static inline int sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

// byte[0]: which motor to move: 0 x, 1 y, 2 z, 3 LED, 4 Laser
// byte[1]: what direction: 1 forward, 0 backward
// byte[2]: how many micro steps - upper 8 bits
// byte[3]: how many micro steps - lower 8 bits

static const int CMD_LENGTH = 9;
static const bool USE_SERIAL_MONITOR = false; // for debug
static const int MSG_LENGTH = 50*10;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
volatile int buffer_tx_ptr;
static const int N_BYTES_POS = 3;

// sensor and data logging
volatile bool flag_log_data = false;
volatile bool flag_read_sensor = false;
uint16_t ch1;
uint16_t ch2;
uint16_t ch3;
volatile uint32_t timestamp = 0; // in number of TIMER_PERIOD_us
# define LOGGING_UNDERSAMPLING  1
volatile int counter_log_data = 0;

// heater
static const int heater_1_pwm = 11;
static const int heater_2_pwm = 12;

// stepper
static const int UART_CS_S0 = 46;
static const int UART_CS_S1 = 47;
#define STEPPER_SERIAL Serial3
static const uint8_t X_driver_ADDRESS = 0b00;
static const float R_SENSE = 0.11f;
TMC2209Stepper X_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);
TMC2209Stepper Y_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);
TMC2209Stepper Z_driver(&STEPPER_SERIAL, R_SENSE, X_driver_ADDRESS);

// driver 1 actuator 1
static const int X_dir = 28;
static const int X_step = 26;
static const int X_en = 36;
static const int X_N_microstepping = 2;
static const long steps_per_mm_X = 78.74*X_N_microstepping; 
constexpr float MAX_VELOCITY_X_mm = 7.62; 
constexpr float MAX_ACCELERATION_X_mm = 100;

// driver 2 - PL25
//static const long steps_per_mm_XY = 30*4; 
//constexpr float MAX_VELOCITY_Y_mm = 25; 
//constexpr float MAX_ACCELERATION_Y_mm = 500; // 50 ms to reach 15 mm/s
static const int Y_dir = 24;
static const int Y_step = 22;
static const int Y_en = 36;
static const int Y_N_microstepping = 2;
static const long steps_per_mm_Y = 78.74*Y_N_microstepping; 
constexpr float MAX_VELOCITY_Y_mm = 7.62; 
constexpr float MAX_ACCELERATION_Y_mm = 100;

static const int Z_dir = 23;
static const int Z_step = 25;
static const int Z_N_microstepping = 2;
static const long steps_per_mm_Z = 82.02*Y_N_microstepping; 
constexpr float MAX_VELOCITY_Z_mm = 18.29; 
constexpr float MAX_ACCELERATION_Z_mm = 100;

AccelStepper stepper_X = AccelStepper(AccelStepper::DRIVER, X_step, X_dir);
AccelStepper stepper_Y = AccelStepper(AccelStepper::DRIVER, Y_step, Y_dir);
AccelStepper stepper_Z = AccelStepper(AccelStepper::DRIVER, Z_step, Z_dir);

long X_commanded_target_position = 0;
bool X_commanded_movement_in_progress = false;
long Y_commanded_target_position = 0;
bool Y_commanded_movement_in_progress = false;
long Z_commanded_target_position = 0;
bool Z_commanded_movement_in_progress = false;

#include <DueTimer.h>
static const float TIMER_PERIOD_us = 5000; // in us

static const float VELOCITY_MAX = 100; // mm/s
static const float ACCELERATION_MAX = 500; // mm/s/s
static const int FULLSTEPS_PER_MM = 40;

void setup() {

  // Initialize Native USB port
  SerialUSB.begin(2000000);     
  while(!SerialUSB);            // Wait until connection is established
  buffer_rx_ptr = 0;

  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
    
  pinMode(X_dir, OUTPUT);
  pinMode(X_step, OUTPUT);

  pinMode(Y_dir, OUTPUT);
  pinMode(Y_step, OUTPUT);

  pinMode(Z_dir, OUTPUT);
  pinMode(Z_step, OUTPUT);

  pinMode(UART_CS_S0, OUTPUT);
  pinMode(UART_CS_S1, OUTPUT);

  // initialize stepper driver
  STEPPER_SERIAL.begin(115200);

  select_driver(1);
  while(!STEPPER_SERIAL);
  X_driver.begin();
  X_driver.I_scale_analog(false);  
  X_driver.rms_current(250,0.2); //I_run and holdMultiplier
  X_driver.microsteps(X_N_microstepping);
  X_driver.pwm_autoscale(true);
  X_driver.TPOWERDOWN(2);
  X_driver.en_spreadCycle(false);
  X_driver.toff(4);
  stepper_X.setPinsInverted(false, false, true);
  stepper_X.setMaxSpeed(MAX_VELOCITY_X_mm*steps_per_mm_X);
  stepper_X.setAcceleration(MAX_ACCELERATION_X_mm*steps_per_mm_X);
  stepper_X.enableOutputs();
  
  select_driver(2);
  while(!STEPPER_SERIAL);
  Y_driver.begin();
  Y_driver.I_scale_analog(false);  
  Y_driver.rms_current(250,0.2); //I_run and holdMultiplier
  Y_driver.microsteps(Y_N_microstepping);
  Y_driver.pwm_autoscale(true);
  Y_driver.TPOWERDOWN(2);
  Y_driver.en_spreadCycle(false);
  Y_driver.toff(4);
  stepper_Y.setPinsInverted(false, false, true);
  stepper_Y.setMaxSpeed(MAX_VELOCITY_Y_mm*steps_per_mm_Y);
  stepper_Y.setAcceleration(MAX_ACCELERATION_Y_mm*steps_per_mm_Y);
  stepper_Y.enableOutputs();

  select_driver(3);
  while(!STEPPER_SERIAL);
  Z_driver.begin();
  Z_driver.I_scale_analog(false);  
  Z_driver.rms_current(400,0.2); //I_run and holdMultiplier
  Z_driver.microsteps(Z_N_microstepping);
  Z_driver.pwm_autoscale(true);
  Z_driver.TPOWERDOWN(2);
  Z_driver.en_spreadCycle(false);
  Z_driver.toff(4);
  stepper_Z.setPinsInverted(false, false, true);
  stepper_Z.setMaxSpeed(MAX_VELOCITY_Z_mm*steps_per_mm_Z);
  stepper_Z.setAcceleration(MAX_ACCELERATION_Z_mm*steps_per_mm_Z);
  stepper_Z.enableOutputs();

  analogReadResolution(12);
  delayMicroseconds(500000);

  // start the timer
  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);
  
}

void loop() {

  // read one meesage from the buffer
  while (SerialUSB.available()) { 
    buffer_rx[buffer_rx_ptr] = SerialUSB.read();
    buffer_rx_ptr = buffer_rx_ptr + 1;
    if (buffer_rx_ptr == CMD_LENGTH) {
      buffer_rx_ptr = 0;
      if(buffer_rx[0]==0)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        int n_microstepping = buffer_rx[4];
        float steps_per_mm = FULLSTEPS_PER_MM*n_microstepping;
        float velocity = VELOCITY_MAX*(float(long(buffer_rx[5])*256 + long(buffer_rx[6]))/65535);
        float acceleration = ACCELERATION_MAX*(float(long(buffer_rx[7])*256 + long(buffer_rx[8]))/65535);
        select_driver(1);
        while(!STEPPER_SERIAL);
        X_driver.begin();
        if(n_microstepping==1)
          X_driver.microsteps(0);
        else
          X_driver.microsteps(n_microstepping);
        stepper_X.setMaxSpeed(velocity*steps_per_mm);
        stepper_X.setAcceleration(acceleration*steps_per_mm);
        
        X_commanded_target_position = (stepper_X.currentPosition()+relative_position);
        stepper_X.moveTo(X_commanded_target_position);
        X_commanded_movement_in_progress = true;
      }
      if(buffer_rx[0]==1)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        int n_microstepping = buffer_rx[4];
        float steps_per_mm = FULLSTEPS_PER_MM*n_microstepping;
        float velocity = VELOCITY_MAX*(float(long(buffer_rx[5])*256 + long(buffer_rx[6]))/65535);
        float acceleration = ACCELERATION_MAX*(float(long(buffer_rx[7])*256 + long(buffer_rx[8]))/65535);
        select_driver(2);
        while(!STEPPER_SERIAL);
        Y_driver.begin();
        if(n_microstepping==1)
          Y_driver.microsteps(0);
        else
          Y_driver.microsteps(n_microstepping);
        stepper_Y.setMaxSpeed(velocity*steps_per_mm);
        stepper_Y.setAcceleration(acceleration*steps_per_mm);
        
        Y_commanded_target_position = (stepper_Y.currentPosition()+relative_position);
        stepper_Y.moveTo(Y_commanded_target_position);
        Y_commanded_movement_in_progress = true;
      }
      if(buffer_rx[0]==2)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        int n_microstepping = buffer_rx[4];
        float steps_per_mm = 100*n_microstepping;
        float velocity = VELOCITY_MAX*(float(long(buffer_rx[5])*256 + long(buffer_rx[6]))/65535);
        float acceleration = ACCELERATION_MAX*(float(long(buffer_rx[7])*256 + long(buffer_rx[8]))/65535);
        select_driver(3);
        while(!STEPPER_SERIAL);
        Z_driver.begin();
        if(n_microstepping==1)
          Z_driver.microsteps(0);
        else
          Z_driver.microsteps(n_microstepping);
        stepper_Z.setMaxSpeed(velocity*steps_per_mm);
        stepper_Z.setAcceleration(acceleration*steps_per_mm);
        
        Z_commanded_target_position = (stepper_Z.currentPosition()+relative_position);
        stepper_Z.moveTo(Z_commanded_target_position);
        Z_commanded_movement_in_progress = true;
      }      
      //break; // exit the while loop after reading one message
      if(buffer_rx[0]==4)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        int n_microstepping = buffer_rx[4];
        float steps_per_mm = FULLSTEPS_PER_MM*n_microstepping;
        float velocity = VELOCITY_MAX*(float(long(buffer_rx[5])*256 + long(buffer_rx[6]))/65535);
        float acceleration = ACCELERATION_MAX*(float(long(buffer_rx[7])*256 + long(buffer_rx[8]))/65535);
        select_driver(1);
        while(!STEPPER_SERIAL);
        X_driver.begin();
        if(n_microstepping==1)
          X_driver.microsteps(0);
        else
          X_driver.microsteps(n_microstepping);
        stepper_X.setMaxSpeed(velocity*steps_per_mm);
        stepper_X.setAcceleration(acceleration*steps_per_mm);

        long start_position = stepper_X.currentPosition();
        long target_position = (stepper_X.currentPosition()+relative_position);
        
        // X_commanded_target_position = (stepper_X.currentPosition()+relative_position);
        // stepper_X.moveTo(X_commanded_target_position);
        // X_commanded_movement_in_progress = true;

        for(int i=0;i<3;i++)
        {
          stepper_X.runToNewPosition(target_position);
          stepper_X.runToNewPosition(start_position);
        }
      }
      if(buffer_rx[0]==6)
      {
        long relative_position = long(buffer_rx[1]*2-1)*(long(buffer_rx[2])*256 + long(buffer_rx[3]));
        int n_microstepping = buffer_rx[4];
        float steps_per_mm = 100*n_microstepping;
        float velocity = VELOCITY_MAX*(float(long(buffer_rx[5])*256 + long(buffer_rx[6]))/65535);
        float acceleration = ACCELERATION_MAX*(float(long(buffer_rx[7])*256 + long(buffer_rx[8]))/65535);
        select_driver(3);
        while(!STEPPER_SERIAL);
        Z_driver.begin();
        if(n_microstepping==1)
          Z_driver.microsteps(0);
        else
          Z_driver.microsteps(n_microstepping);
        stepper_Z.setMaxSpeed(velocity*steps_per_mm);
        stepper_Z.setAcceleration(acceleration*steps_per_mm);

        long start_position = stepper_Z.currentPosition();
        long target_position = (stepper_Z.currentPosition()+relative_position);
        
        // X_commanded_target_position = (stepper_X.currentPosition()+relative_position);
        // stepper_X.moveTo(X_commanded_target_position);
        // X_commanded_movement_in_progress = true;

        for(int i=0;i<3;i++)
        {
          stepper_Z.runToNewPosition(target_position);
          stepper_Z.runToNewPosition(start_position);
        }
      }
      if(buffer_rx[0]==7)
        analogWrite(heater_1_pwm,buffer_rx[1]);
      if(buffer_rx[0]==8)
        analogWrite(heater_2_pwm,buffer_rx[1]);
    }
  }


  // check if commanded position has been reached
  if(X_commanded_movement_in_progress && stepper_X.currentPosition()==X_commanded_target_position)
    X_commanded_movement_in_progress = false;
  // move motors
  if(X_commanded_movement_in_progress)
    stepper_X.run();
  
  if(Y_commanded_movement_in_progress && stepper_Y.currentPosition()==Y_commanded_target_position)
    Y_commanded_movement_in_progress = false;
  // move motors
  if(Y_commanded_movement_in_progress)
    stepper_Y.run();

  if(Z_commanded_movement_in_progress && stepper_Z.currentPosition()==Z_commanded_target_position)
    Z_commanded_movement_in_progress = false;
  // move motors
  if(Z_commanded_movement_in_progress)
    stepper_Z.run();

  if (flag_read_sensor)
  {
    ch1 = analogRead(A4);
    ch2 = analogRead(A6);
    ch3 = analogRead(A8);
    flag_read_sensor = false;
  }

  if (flag_log_data)
  {
    flag_log_data = false;
    
    // field 1: time
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 24);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 16);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp %256);

    // field 2 ch1
    buffer_tx[buffer_tx_ptr++] = byte(ch1 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(ch1 % 256);

    // field 3 ch2
    buffer_tx[buffer_tx_ptr++] = byte(ch2 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(ch2 % 256);

    // field 4 ch3
    buffer_tx[buffer_tx_ptr++] = byte(ch3 >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(ch3 % 256);

    if (buffer_tx_ptr == MSG_LENGTH)
    {
      buffer_tx_ptr = 0;
      if(USE_SERIAL_MONITOR)
      {
        SerialUSB.println(ch1);
      }
      else
        SerialUSB.write(buffer_tx, MSG_LENGTH);
    }
  }
}


/***************************************************************************************************/
/******************************** timer interrupt handling routine *********************************/
/***************************************************************************************************/
void timer_interruptHandler()
{
  timestamp = timestamp + 1;

  // read sensor value
  flag_read_sensor = true;
  
  // send data to host computer
  counter_log_data = counter_log_data + 1;
  if (counter_log_data >= LOGGING_UNDERSAMPLING)
  {
    counter_log_data = 0;
    flag_log_data = true;
  }
}

// utils
long signed2NBytesUnsigned(long signedLong,int N)
{
  long NBytesUnsigned = signedLong + pow(256L,N)/2;
  //long NBytesUnsigned = signedLong + 8388608L;
  return NBytesUnsigned;
}
void select_driver(int id)
{
  if(id==1)
  {
    digitalWrite(UART_CS_S0, LOW);
    digitalWrite(UART_CS_S1, LOW);
  }
  if(id==2)
  {
    digitalWrite(UART_CS_S0, HIGH);
    digitalWrite(UART_CS_S1, LOW);
  }
  if(id==3)
  {
    digitalWrite(UART_CS_S0, LOW);
    digitalWrite(UART_CS_S1, HIGH);
  }
  if(id==4)
  {
    digitalWrite(UART_CS_S0, HIGH);
    digitalWrite(UART_CS_S1, HIGH);
  }
}
