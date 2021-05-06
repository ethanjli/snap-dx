#include <AccelStepper.h>  // Used by StepperMotor

// The HAL classes define low-level device drivers.
// These drivers should never call the delay function!

class DigitalOutput {
   public:
    DigitalOutput(uint8_t pin, bool active_low)
        : pin_(pin),
          active_low_(active_low),
          inactive_level_(active_low),
          active_level_(!active_low) {}

    // Always returns true
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

class DigitalInput {
   public:
    DigitalInput(uint8_t pin, bool active_low, bool pull_up)
        : pin_(pin), active_level_(!active_low), pull_up_(pull_up) {}

    // Always returns true
    bool setup() {
        if (pull_up_) {
            pinMode(pin_, INPUT_PULLUP);
        } else {
            pinMode(pin_, INPUT);
        }
        return true;
    }

    bool is_active() { digitalRead(pin_) == active_level_; }

   private:
    const uint8_t pin_;
    const bool active_level_;
    const bool pull_up_;
};

class LCD {
   public:
    // Always returns true
    bool setup() {
        Serial.begin(115200);
        return true;
    }
};

class ESP32Camera {
   public:
    enum class Result {
        off = 0,       // 0/0: camera off
        positive = 1,  // 0/1: camera on, positive result
        negative = 2,  // 1/0: camera on, negative result
        invalid = 3    // 1/1: camera on, indeterminate result or invalid state
    };

    ESP32Camera(uint8_t input_1, uint8_t input_2)
        : input_1_(input_1, false, false), input_2_(input_2, false, false) {}

    // Returns true if the ESP32 produces the expected output
    bool setup() {
        if (!input_1_.setup()) {
            return false;
        }

        if (!input_2_.setup()) {
            return false;
        }

        // TODO: wait for ESP32 to produce output (maybe a fixed delay)
        return read() == Result::invalid;
    }

    Result read() {
        bool input_1_reading = input_1_.is_active();
        bool input_2_reading = input_2_.is_active();
        if (input_1_reading && input_2_reading) {
            return Result::invalid;
        } else if (input_1_reading) {
            return Result::negative;
        } else if (input_2_reading) {
            return Result::positive;
        } else {
            return Result::off;
        }
    }

   private:
    DigitalInput input_1_;
    DigitalInput input_2_;
};

class Thermistor {
   public:
    Thermistor(uint8_t sampling_pin, uint8_t reference_pin)
        : sampling_pin_(sampling_pin), reference_pin_(reference_pin) {}

    // Always returns true
    bool setup() {
        analogReadResolution(12);  // Requires Arduino Due, Zero, or MKR
        // TODO: we could take a reading and check if it's within an expected
        // range (e.g. 0 deg C to 100 deg C) and return false otherwisea.
        return true;
    }

    float temperature() {
        float frac =
            1.0 * analogRead(sampling_pin_) / analogRead(reference_pin_);
        // Value of the thermistor
        float R_th = R * frac / (1 - frac);             // Ohm
        float T = 1 / (A + B * log(R_th / R_0)) - T_0;  // deg C

        return T;
    }

   private:
    // Value of the resistor in the thermistor circuit
    static constexpr float R = 1977;      // Ohm
    static constexpr float A = 0.003354;  // K^-1
    static constexpr float B = 0.000289;  // K^-1
    static constexpr float R_0 = 10000;   // Ohm
    static constexpr float T_0 = 273.15;  // K

    const uint8_t sampling_pin_;
    const uint8_t reference_pin_;
};

class StepperMotor {
   public:
    StepperMotor(uint8_t dir_pin, uint8_t step_pin, uint8_t en_pin)
        : dir_pin_(dir_pin),
          step_pin_(step_pin),
          enable_(en_pin, true),
          stepper_(AccelStepper::DRIVER, step_pin_, dir_pin_) {}

    // Should always return true
    bool setup() {
        pinMode(dir_pin_, OUTPUT);
        pinMode(step_pin_, OUTPUT);
        if (!enable_.setup()) {
            // Should never happen
            return false;
        }

        enable_.deactivate();
        stepper_.setPinsInverted(false, false, true);
        stepper_.setMaxSpeed(max_velocity * distance_resolution);
        stepper_.setAcceleration(acceleration * distance_resolution);
        stepper_.enableOutputs();
        return true;
    }
    void update() {
        if (remaining_estimated_displacement() == 0) {
            stop_move();
            return;
        }

        stepper_.run();
    }

    void start_move(float displacement, float target_velocity) {
        stepper_.setMaxSpeed(target_velocity * distance_resolution);
        stepper_.setAcceleration(acceleration * distance_resolution);
        target_position_ = stepper_.currentPosition() + displacement;
        stepper_.moveTo(target_position_);
        moving_ = true;
        enable_.activate();
    }
    void stop_move() {
        moving_ = false;
        enable_.deactivate();
    }
    long remaining_estimated_displacement() { return stepper_.distanceToGo(); }

   private:
    static const int microsteps = 16;
    static const long distance_resolution = 500 * microsteps;  // steps/mm
    static constexpr float max_velocity = 18.29;               // mm/s
    static constexpr float acceleration = 10;                  // mm/s/s

    const uint8_t dir_pin_;
    const uint8_t step_pin_;
    DigitalOutput enable_;
    AccelStepper stepper_;
    long target_position_ = 0;
    bool moving_ = false;
};