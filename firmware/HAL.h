#include <AccelStepper.h>  // Used by StepperMotor

// The HAL classes define low-level device drivers.

class DigitalOutput {
   public:
    DigitalOutput(uint8_t pin, bool active_low)
        : pin_(pin),
          active_low_(active_low),
          inactive_level_(active_low),
          active_level_(!active_low) {}

    virtual bool setup() {
        pinMode(pin_, OUTPUT);
        deactivate();
        return true;
    }

    bool activate() { digitalWrite(pin_, active_level_); }
    bool deactivate() { digitalWrite(pin_, inactive_level_); }

   private:
    const uint8_t pin_;
    const bool active_low_;
    const bool inactive_level_;
    const bool active_level_;
};

class LCD {};
class Button {};
class ESP32Camera {};
class Thermistor {
   public:
    Thermistor(uint8_t sampling_pin, uint8_t reference_pin)
        : sampling_pin_(sampling_pin), reference_pin_(reference_pin) {}

    bool setup() {
        analogReadResolution(12);  // Requires Arduino Due, Zero, or MKR
        // Do we need to wait a bit of time for reading to stabilize?
        // Is it safe to re-sample the reference pin for each thermistor's
        // setup routine?
        reference_value_ = analogRead(reference_pin_);
        // How does the Arduino sketch use ch1, ch2, and ch3?
        return true;
    }

   private:
    const uint8_t sampling_pin_;
    const uint8_t reference_pin_;
    int reference_value_;
};
class Fan {};
class StepperMotor {
   public:
    StepperMotor(uint8_t dir_pin, uint8_t step_pin, uint8_t en_pin)
        : dir_pin_(dir_pin),
          step_pin_(step_pin),
          en_pin_(en_pin),
          stepper_(AccelStepper::DRIVER, step_pin_, dir_pin_) {}
    bool setup() {
        pinMode(dir_pin_, OUTPUT);
        pinMode(step_pin_, OUTPUT);
        pinMode(en_pin_, OUTPUT);
        digitalWrite(en_pin_, HIGH);
        stepper_.setPinsInverted(false, false, true);
        stepper_.setMaxSpeed(max_velocity * distance_resolution);
        stepper_.setAcceleration(max_acceleration * distance_resolution);
        stepper_.enableOutputs();
        return true;
    }

   private:
    static const int microsteps = 16;
    static const long distance_resolution = 500 * microsteps;  // steps/mm
    static constexpr float max_velocity = 18.29;               // mm/s
    static constexpr float max_acceleration = 100;             // mm/s/s

    const uint8_t dir_pin_;
    const uint8_t step_pin_;
    const uint8_t en_pin_;
    AccelStepper stepper_;
};
class LimitSwitch {
   public:
    LimitSwitch(uint8_t pin) : pin_(pin) {}

    bool setup() {
        pinMode(pin_, INPUT_PULLUP);
        // The prototype sketch uses interrupts, but if we can do polling that
        // may be better; we also get debouncing this way, which should reduce
        // weird behavior in the motor
        return true;
    }

   private:
    const uint8_t pin_;
};
class Solenoid {};
class SDCard {};