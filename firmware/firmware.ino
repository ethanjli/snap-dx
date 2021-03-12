#include <TMCStepper.h>
#include <TMCStepper_UTILITY.h>
#include <AccelStepper.h>
#include <DueTimer.h>


///////////////////////////////////////////////////////////
/////////////Serial commands//////////////////////////////
//////////////////////////////////////////////////////////
// byte[0]: what is controlled: 2 z, 0 heater 1, 1 heater 2, 3, tickler

// for motor (byte[0]==2)
// byte[1]: what direction: 1 forward, 0 backward
// byte[2]: how many micro steps - upper 8 bits
// byte[3]: how many micro steps - lower 8 bits

// for heaters (byte[0]==0|1)
// byte[1]: on if>0; off if ==0

// for tickler (byte[0]==3)
// byte[1]: on/off



///////////////////////////////////////////////////////////
/////////////Init variables//////////////////////////////
//////////////////////////////////////////////////////////


// heater 1
static const int heater_1 = 44;
float temp_1 = 0; // temp heater 1
unsigned long temp_1_interval = 500UL; // temperature update every second
unsigned long temp_1_t0 = 0;
float temp_1_setpoint = 95; //XX C temperature setpoint
bool heater_1_act = false; // is the heater activated for temperature regulation
unsigned long heater_1_onTime = 0; // time since heater switched ON
unsigned long heater_1_offTime = 0; // time since heater switched OFF
unsigned long heater_1_interval_on = 1000UL * 10 ; //last number is number of seconds per interval
unsigned long heater_1_interval_off = 1000UL * 20; //last number is number of seconds per interval


// heater 2
static const int heater_2 = 42;
float temp_2 = 0; // temp heater 2
unsigned long temp_2_interval = 1000UL; // temperature update every second
unsigned long temp_2_t0 = 0;
float temp_2_setpoint = 60; //XX C temperature setpoint
bool heater_2_act = false; // is the heater activated for temperature regulation
unsigned long heater_2_onTime = 0; // time since heater switched ON
unsigned long heater_2_offTime = 0; // time since heater switched OFF
unsigned long heater_2_interval_on = 1000UL * 5 ; //last number is number of seconds per interval
unsigned long heater_2_interval_off = 1000UL * 5; //last number is number of seconds per interval

// tickler
static const int tickler = 40; //tickler pin
bool tickler_on = false; // is tickler on? current status

// solenoid
static const int solenoid = 38; //solenoid pin
bool solenoid_on = false; // is solenoid on? current status

// Stepper motor
static const int Z_step = 35; // test
static const int Z_dir = 33; // test
static const int Z_en = 31; // test
static const int N_microsteps=16;
static const long steps_per_mm_Z = 500*N_microsteps;
static const int FULLSTEPS_PER_MM = 40;
constexpr float MAX_VELOCITY_Z_mm = 18.29;
constexpr float MAX_ACCELERATION_Z_mm = 100;
AccelStepper stepper_Z = AccelStepper(AccelStepper::DRIVER, Z_step, Z_dir);
long Z_commanded_target_position = 0;
bool Z_commanded_movement_in_progress = false;
static const float VELOCITY_MAX = 100; // mm/s
static const float ACCELERATION_MAX = 500; // mm/s/s


// limit switches for Z axis
const byte switch_top = 22;
const byte switch_bottom = 23;
volatile bool flag_disable_motorUp = false;
volatile bool flag_disable_motorDown = false;
bool switch_top_off = false;
bool switch_bottom_off = false;

// Serial connection
static const int CMD_LENGTH = 9;
static const bool USE_SERIAL_MONITOR = false; // for debug
static const int MSG_LENGTH = 50 * 10;
byte buffer_rx[500];
byte buffer_tx[MSG_LENGTH];
volatile int buffer_rx_ptr;
volatile int buffer_tx_ptr;
static const int N_BYTES_POS = 3;

// sensor and data logging
static const int temp_ch1 = A0; // ref pin
static const int temp_ch2 = A1; // thermistor 1
static const int temp_ch3 = A2; // thermistor 2

volatile bool flag_log_data = false;
volatile bool flag_read_sensor = false;
uint16_t ch1;
uint16_t ch2;
uint16_t ch3;
volatile uint32_t timestamp = 0; // in number of TIMER_PERIOD_us
# define LOGGING_UNDERSAMPLING  1
volatile int counter_log_data = 0;
static const float TIMER_PERIOD_us = 5000; // in us

///////////////////////////////////////////////////////////
//////////////////////setup //////////////////////////////
//////////////////////////////////////////////////////////

void setup() {

  // Initialize Native USB port
  SerialUSB.begin(2000000);
  while (!SerialUSB);           // Wait until connection is established
  buffer_rx_ptr = 0;

  pinMode(13, OUTPUT); //Mel     not sure what that one is for...
  digitalWrite(13, LOW);


  // initialize limit switches
  pinMode(switch_top, INPUT_PULLUP);
  pinMode(switch_bottom, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(switch_top), interrupt_motorUp, RISING);
  attachInterrupt(digitalPinToInterrupt(switch_bottom), interrupt_motorDown, RISING);

  // initialize stepper
  pinMode(Z_dir, OUTPUT);
  pinMode(Z_step, OUTPUT);
  pinMode(Z_en, OUTPUT);
  digitalWrite(Z_en, HIGH);
  stepper_Z.setPinsInverted(false, false, true);
  stepper_Z.setMaxSpeed(MAX_VELOCITY_Z_mm * steps_per_mm_Z);
  stepper_Z.setAcceleration(MAX_ACCELERATION_Z_mm * steps_per_mm_Z);
  stepper_Z.enableOutputs();

  analogReadResolution(12);
  delayMicroseconds(500000);


  // start the timer
  Timer3.attachInterrupt(timer_interruptHandler);
  Timer3.start(TIMER_PERIOD_us);


  //  initialize heater
  pinMode(heater_1, OUTPUT);
  digitalWrite(heater_1, HIGH); //HIGH means heater OFF
  pinMode(heater_2, OUTPUT);
  digitalWrite(heater_2, HIGH); //HIGH means heater OFF


  // initialise temperature measurement
  ch1 = analogRead(temp_ch1);
  ch2 = analogRead(temp_ch2);
  ch3 = analogRead(temp_ch3);
  temp_1 = get_temp(ch1, ch3);
  temp_1_t0 = millis();

  temp_2 = get_temp(ch1, ch2);
  temp_2_t0 = millis();

  // initialize tickler
  pinMode(tickler, OUTPUT);
  digitalWrite(tickler, HIGH); //HIGH means heater OFF

}

///////////////////////////////////////////////////////////
//////////////////////MAIN loop //////////////////////////
//////////////////////////////////////////////////////////

void loop() {

  // read one meesage from the buffer
  while (SerialUSB.available()) {
    buffer_rx[buffer_rx_ptr] = SerialUSB.read();
    buffer_rx_ptr = buffer_rx_ptr + 1;
    if (buffer_rx_ptr == CMD_LENGTH)
    {
      buffer_rx_ptr = 0;

      if (buffer_rx[0] == 2) // translation motor
      {
        long relative_position = long(buffer_rx[1] * 2 - 1) * (long(buffer_rx[2]) * 256 + long(buffer_rx[3]));
        float steps_per_mm = steps_per_mm_Z;
        float velocity = VELOCITY_MAX * (float(long(buffer_rx[5]) * 256 + long(buffer_rx[6])) / 65535);
        float acceleration = ACCELERATION_MAX * (float(long(buffer_rx[7]) * 256 + long(buffer_rx[8])) / 65535);
        //    select_driver(3);
        //        while (!STEPPER_SERIAL);
        //        Z_driver.begin();//
        //        if (n_microstepping == 1)
        //          Z_driver.microsteps(0);
        //        else
        //          Z_driver.microsteps(n_microstepping);
        stepper_Z.setMaxSpeed(velocity * steps_per_mm);
        stepper_Z.setAcceleration(acceleration * steps_per_mm);

        Z_commanded_target_position = (stepper_Z.currentPosition() + relative_position);
        stepper_Z.moveTo(Z_commanded_target_position);
        Z_commanded_movement_in_progress = true;
        digitalWrite(Z_en, LOW);
      }

      if (buffer_rx[0] == 3) //tickler
      {
        if (buffer_rx[1] > 0) { //activate tickler
          digitalWrite(tickler, LOW);//stwitch tickler ON
          tickler_on = true;
        }
        else {//deactivate heater
          digitalWrite(tickler, HIGH);//stwitch tickler OFF
          tickler_on = false;
        }
      }


      if (buffer_rx[0] == 0) //heater 1
      {
        if (buffer_rx[1] > 0) { //activate heater1
          if (heater_1_onTime == 0) { //if the heater was off, start it
            heater_1_act = true;
            digitalWrite(heater_1, LOW);
            heater_1_onTime = millis();
            heater_1_offTime = 0;
          }
        }
        else {//deactivate heater
          digitalWrite(heater_1, HIGH);
          heater_1_onTime = 0;
          heater_1_offTime = 0;
          heater_1_act = false;
        }
      }

      if (buffer_rx[0] == 1) //heater 2
      {
        if (buffer_rx[1] > 0) { //activate heater1
          if (heater_2_onTime == 0) { //if the heater was off, start it
            heater_2_act = true;
            digitalWrite(heater_2, LOW);
            heater_2_onTime = millis();
            heater_2_offTime = 0;
          }
        }
        else {//deactivate heater
          digitalWrite(heater_2, HIGH);
          heater_2_onTime = 0;
          heater_2_offTime = 0;
          heater_2_act = false;
        }
      }
    }

  }


  if (millis() - temp_1_t0 > temp_1_interval) //measure temperature every temp_1_interval
  {
    temp_1 = get_temp(ch1, ch2);
    temp_1_t0 = millis();
    temp_2 = get_temp(ch1, ch3);
    temp_2_t0 = millis();
  }


  if (heater_1_act == true) {
    if (heater_1_onTime > 0 && millis() - heater_1_onTime > heater_1_interval_on) //if it has been on for too long, switch off
    {
      heater_1_onTime = 0;
      digitalWrite(heater_1, HIGH);//stop heating
      heater_1_offTime = millis();
    }

    if (heater_1_onTime > 0 && temp_1 > temp_1_setpoint) //switch off it too warm
    {
      heater_1_onTime = 0;
      digitalWrite(heater_1, HIGH);//stop heating
      heater_1_offTime = millis();
    }

    if (heater_1_offTime > 0 && millis() - heater_1_offTime > heater_1_interval_off) //if it has been off for too long, switch off
    {
      heater_1_offTime = 0;
      digitalWrite(heater_1, LOW);//start heating
      heater_1_onTime = millis();
    }

  }


  if (heater_2_act == true) {
    if (heater_2_onTime > 0 && millis() - heater_2_onTime > heater_2_interval_on) //if it has been on for too long, switch off
    {
      heater_2_onTime = 0;
      digitalWrite(heater_2, HIGH);//stop heating
      heater_2_offTime = millis();
    }

    if (heater_2_onTime > 0 && temp_2 > temp_2_setpoint) //switch off it too warm
    {
      heater_2_onTime = 0;
      digitalWrite(heater_2, HIGH);//stop heating
      heater_2_offTime = millis();
    }

    if (heater_2_offTime > 0 && millis() - heater_2_offTime > heater_2_interval_off) //if it has been off for too long, switch off
    {
      heater_2_offTime = 0;
      digitalWrite(heater_2, LOW);//start heating
      heater_2_onTime = millis();
    }

  }



  if (Z_commanded_movement_in_progress && stepper_Z.currentPosition() == Z_commanded_target_position)
  {
    Z_commanded_movement_in_progress = false;
    digitalWrite(Z_en, HIGH);
  }
  // move motors
  if (Z_commanded_movement_in_progress) {
    switch_top_off = digitalRead(switch_top);
    switch_bottom_off = digitalRead(switch_bottom);
    if ((stepper_Z.distanceToGo() > 0) && (!switch_top_off))
    {
      stepper_Z.stop();
    }
    if ((stepper_Z.distanceToGo() < 0) && (!switch_bottom_off))
    {
      stepper_Z.stop();
    }
    stepper_Z.run();
  }




  if (flag_read_sensor)
  {
    ch1 = analogRead(temp_ch1);
    ch2 = analogRead(temp_ch2);
    ch3 = analogRead(temp_ch3);
    flag_read_sensor = false;
  }

  if (flag_log_data)
  {
    flag_log_data = false;

    // field 1: time
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 24);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 16);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp >> 8);
    buffer_tx[buffer_tx_ptr++] = byte(timestamp % 256);

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
      if (USE_SERIAL_MONITOR)
      {
        SerialUSB.println(ch1);
      }
      else
        SerialUSB.write(buffer_tx, MSG_LENGTH);
    }
  }
}


/***************************************************************************************************/
/********************************************* functions **********************************************/
/***************************************************************************************************/

float get_temp(int daq1, int daq2)
{
  float R = 1977;
  float frac = 0;
  float Rth = 0;
  float T;

  frac = (float) daq2 / (float) daq1;
  Rth = R * frac / (1 - frac);
  T = 1 / (0.003354 + 0.000289 * log(Rth / 10000)) - 273.15;


  return T;

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

// switch limit interrupt for motor - Mel
void interrupt_motorUp() {
  flag_disable_motorUp = true;
}
void interrupt_motorDown() {

  flag_disable_motorDown = true;
}

/***************************************************************************************************/
/********************************************* UTILS **********************************************/
/***************************************************************************************************/

long signed2NBytesUnsigned(long signedLong, int N)
{
  long NBytesUnsigned = signedLong + pow(256L, N) / 2;
  //long NBytesUnsigned = signedLong + 8388608L;
  return NBytesUnsigned;
}

static inline int sgn(int val) {
  if (val < 0) return -1;
  if (val == 0) return 0;
  return 1;
}
