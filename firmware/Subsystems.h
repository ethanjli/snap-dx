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
    bool setup() {
        // switch on fan
        return true;
    }

    void set_temperature(int temperature) {}
    void start_control() {}
    void stop_control() {}
    bool temperature_converged(int temperature) {}

   private:
    Heater heater_1_;
    Heater heater_2_;
    Thermistor thermistor_1_;
    Thermistor thermistor_2_;
    TemperatureControlLoop control_loop_;
    Fan fan_;
};

class MotionController {
   public:
    bool setup() {
        move_to(0);
        // Are there any potential hardware faults to catch, e.g. motor can't
        // move?
        return true;
    }

    void move_to(uint32_t position) {}

   private:
    StepperMotor motor_;
    LimitSwitch switch_1_;
    LimitSwitch switch_2_;
};

class Mixer {
   public:
    bool setup() {}

    void start_mixing() {}
    void stop_mixing() {}

   private:
    Tickler tickler_;
};

class Door {
   public:
    bool setup() {
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
    Solenoid solenoid_;
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