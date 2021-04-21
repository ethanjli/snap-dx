#include "ControlAlgorithms.h"
#include "HAL.h"

// The Subsystems classes define high-level interfaces for
// functionally-related collections of low-level devices.

class UserInterface {
   public:
    enum class ButtonsState { primary, secondary, both, neither };
    bool setup() {}

    void report_initialization_failure() {}
    void print_message() {}
    void label_buttons() {}
    // Do we need debouncing of button reads?
    ButtonsState read_buttons() {}

   private:
    LCD display_;
    Button button_1_;
    Button button_2_;
};

class ColorDetector {
   public:
    bool setup() {
        // wait for ESP32 to produce output (maybe a fixed delay?)
        // check for ESP32 to output 0/1
        // "check if camera is working, lights are functioning"
        // but the schematic does not have lights!
    }

    void read() {}

   private:
    ESP32Camera camera_;
};

class ThermalController {
   public:
    ThermalController()
        : heater_1_(44, true),
          heater_2_(42, true),
          thermistor_1_(A1, A0),
          thermistor_2_(A2, A0) {}
    bool setup() {
        if (!thermistor_1_.setup()) {
            return false;
        }

        if (!thermistor_2_.setup()) {
            return false;
        }

        if (!heater_1_.setup()) {
            return false;
        }

        if (!heater_2_.setup()) {
            return false;
        }

        // switch on fan
        return true;
    }

    void set_temperature(int temperature) {}
    void start_control() {}
    void stop_control() {}
    bool temperature_converged(int temperature) {}

   private:
    DigitalOutput heater_1_;
    DigitalOutput heater_2_;
    Thermistor thermistor_1_;
    Thermistor thermistor_2_;
    TemperatureControlLoop control_loop_;
    Fan fan_;
};

class MotionController {
   public:
    MotionController()
        : stepper_(33, 35, 31), switch_top_(22), switch_bottom_(23) {}

    bool setup() {
        if (!switch_top_.setup()) {
            return false;
        }

        if (!switch_bottom_.setup()) {
            return false;
        }

        if (!stepper_.setup()) {
            return false;
        }

        move_to(0);
        // Are there any potential hardware faults to catch, e.g. motor can't
        // move?
        return true;
    }

    // Do we need debouncing of limit switch reads?
    void move_to(uint32_t position) {}

   private:
    StepperMotor stepper_;
    LimitSwitch switch_top_;
    LimitSwitch switch_bottom_;
};

class Mixer {
   public:
    Mixer() : tickler_(40, true) {}

    bool setup() {
        if (!tickler_.setup()) {
            return false;
        }
    }

    void start_mixing() {}
    void stop_mixing() {}

   private:
    DigitalOutput tickler_;
};

class Door {
   public:
    Door()
        : solenoid_(38, true /*is the solenoid an active-low output?*/),
          lock_switch_(0 /*what pin is used for the door's limit switch?*/) {}
    bool setup() {
        if (!lock_switch_.setup()) {
            return false;
        }

        if (!is_open()) {
            lock();
            return true;
        } else {
            // apparently we just "beep alarm"
            // but the schematic does not have beeper!
            return false;
        }
    }

    void lock() {}
    void unlock() {}
    bool is_open() {}

   private:
    DigitalOutput solenoid_;
    LimitSwitch lock_switch_;
};

class EventLogger {
   public:
    bool setup() {
        // check if SD card is present
        // ensure that SD card has the proper data structure/filesystem
    }
    void log_initialization_failure() {}
    void log() {}

   private:
    SDCard sd_card_;
};