#pragma once

#include "Algorithms.h"
#include "HAL.h"
#include "Util.h"

bool past_timeout(unsigned long previous_time, unsigned long timeout) {
    return past_timeout(millis(), previous_time, timeout);
}

// The Subsystems classes define high-level interfaces for
// functionally-related collections of low-level devices.

class DebouncedSwitch {
   public:
    enum class State : uint8_t {
        bouncing = 255,  // switch is transitioning between active and inactive
        inactive = 0,    // switch was deactivated before the last read
        deactivated,     // switch was just deactivated in the last read
        activated,       // switch was just activated down in the last read
        active           // switch was activated down before the last read
    };

    const uint8_t pin;  // only for debugging

    DebouncedSwitch(uint8_t pin, bool active_low, bool pull_up)
        : pin(pin), button_(pin, active_low, pull_up) {}

    // Should always return true
    bool setup() { return button_.setup(); }
    // Sample the switch and apply debouncing.
    void update() {
        bool debounced;
        switch (
            debouncer_.transform(button_.is_active(), millis(), debounced)) {
            case Debouncer::Status::ok:
                break;
            case Debouncer::Status::unstable:
                if (state_ != State::bouncing) {
                    SerialUSB.print("DebouncedSwitch(");
                    SerialUSB.print(pin);
                    SerialUSB.println(").update: bouncing too much!");
                }
                state_ = State::bouncing;
                return;
            default:
                return;
        }

        EdgeDetector::State state;
        edge_detector_.transform(debounced, state);
        switch (state) {
            case EdgeDetector::State::rising_edge:
                SerialUSB.print("DebouncedSwitch(");
                SerialUSB.print(pin);
                SerialUSB.println(").update: activated!");
                state_ = State::activated;
                break;
            case EdgeDetector::State::falling_edge:
                SerialUSB.print("DebouncedSwitch(");
                SerialUSB.print(pin);
                SerialUSB.println(").update: deactivated!");
                state_ = State::deactivated;
                break;
            default:
                break;
        }
    }
    State getState() { return state_; }
    // Read the debouncing results, apply edge detection, and return result.
    // Returns activated/deactivated if the switch has been
    // activated/deactivated since the last call to read(); otherwise, returns
    // bouncing/active/inactive.
    State read() {
        State current_state = state_;
        switch (state_) {
            case State::activated:
                // SerialUSB.print("DebouncedSwitch(");
                // SerialUSB.print(pin);
                // SerialUSB.println(").read: consumed activation!");
                state_ = State::active;
                break;
            case State::deactivated:
                // SerialUSB.print("DebouncedSwitch(");
                // SerialUSB.print(pin);
                // SerialUSB.println(").read: consumed deactivation!");
                state_ = State::inactive;
                break;
            default:
                break;
        }
        return current_state;
    }

   private:
    DigitalInput button_;
    Debouncer debouncer_;
    EdgeDetector edge_detector_;
    State state_ = State::bouncing;
};

class UserInterface {
   public:
    enum class ButtonsState : uint8_t { primary = 0, secondary, both, neither };

    DebouncedSwitch primary;
    DebouncedSwitch secondary;

    UserInterface(uint8_t primary_button, uint8_t secondary_button)
        : primary(primary_button, true, true),
          secondary(secondary_button, true, true) {}

    // Should always return true
    bool setup() {
        if (!primary.setup()) {
            // Should never happen
            return false;
        }

        if (!secondary.setup()) {
            // Should never happen
            return false;
        }

        if (!display_.setup()) {
            // Should never happen, at least for the current mock of the LCD
            // screen which just initializes Serial
            return false;
        }

        return true;
    }
    void update() {
        primary.update();
        secondary.update();
    }

    // Print a C-style string as the UI message
    // Messages may be up to ??? characters long; beyond that, they'll
    // need to be animated (e.g. in a marquee style)
    void print_message(const char *message) {
        SerialUSB.print("----------------------------------------");
        SerialUSB.print("----------------------------------------> [ ");
        SerialUSB.print(message);
        SerialUSB.println(" ]");
    }

    // Clear button labels
    void label_buttons() {
        SerialUSB.print("----------------------------------------");
        SerialUSB.print("----------------------------------------- ");
        SerialUSB.println("( -- | -- )");
    }
    // Set primary label, clear secondary label; takes C-style strings
    void label_buttons(const char *primary) {
        SerialUSB.print("----------------------------------------");
        SerialUSB.print("----------------------------------------- ");
        SerialUSB.print("( ");
        SerialUSB.print(primary);
        SerialUSB.println(" | -- )");
    }
    // Set primary & secondary labels; takes C-style strings
    void label_buttons(const char *primary, const char *secondary) {
        SerialUSB.print("----------------------------------------");
        SerialUSB.print("----------------------------------------- ");
        SerialUSB.print("( ");
        SerialUSB.print(primary);
        SerialUSB.print(" | ");
        SerialUSB.print(secondary);
        SerialUSB.println(" )");
    }

   private:
    LCD display_;
};

class ThermalController {
   public:
    Thermistor thermistor;

    ThermalController(
        uint8_t heater,
        uint8_t thermistor_sampler,
        uint8_t thermistor_reference,
        unsigned long pulse_on_max_duration,
        unsigned long pulse_off_duration)
        : heater_(heater, true),
          thermistor(thermistor_sampler, thermistor_reference),
          control_loop_(pulse_on_max_duration, pulse_off_duration) {}

    // Should always return true. Initializes the heater as deactivated.
    bool setup() {
        if (!thermistor.setup()) {
            // Should never happen, at least for the current implementation of
            // thermistor which doesn't check if it gives a nonsensical reading
            // upon startup
            return false;
        }

        if (!heater_.setup()) {
            // Should never happen
            return false;
        }

        stop_control();
        return true;
    }
    // Update the heater controller once. Returns immediately.
    void update() {
        if (control_loop_.update(millis(), thermistor.temperature())) {
            heater_.activate();
        } else {
            heater_.deactivate();
        }
    }

    // Start controlling the heater. Returns immediately.
    void start_control(float setpoint) {
        control_loop_.start_control(setpoint);
    }
    // Stop controlling the heater, and turn it off. Returns immediately.
    void stop_control() {
        control_loop_.stop_control();
        heater_.deactivate();
    }

   private:
    static const unsigned long update_interval = 500;  // ms

    DigitalOutput heater_;
    TemperatureControlLoop
        control_loop_;  // like bang-bang with a constant duty cycle
    float temperature_ = 0;
};

class MotionController {
   public:
    DebouncedSwitch switch_top;
    DebouncedSwitch switch_bottom;

    MotionController(
        uint8_t dir,
        uint8_t step,
        uint8_t en,
        uint8_t switch_top,
        uint8_t switch_bottom)
        : stepper_(dir, step, en),
          switch_top(
              switch_top,
              true /* TODO: low when pressed? */,
              true /* TODO: pullup input? */),
          switch_bottom(
              switch_bottom,
              true /* TODO: low when pressed? */,
              true /* TODO: pullup input? */) {}

    // Returns false if unable to go to home position (bottom limit)
    bool setup() {
        if (!switch_top.setup()) {
            // Should never happen
            return false;
        }

        if (!switch_bottom.setup()) {
            // Should never happen
            return false;
        }

        if (!stepper_.setup()) {
            // Should never happen
            return false;
        }

        // Try to go to home position (bottom limit)
        const float move_speed = 5;                   // mm/s
        const unsigned long move_timeout = 5 * 1000;  // timeout of 5 sec
        start_move(-1 * move_speed);                  // move to the bottom
        const unsigned long start_time = millis();
        while (true) {
            update();
            if (switch_bottom.read() == DebouncedSwitch::State::active) {
                return true;
            }

            if (past_timeout(start_time, move_timeout)) {
                return false;
            }
        }
    }
    // Update stepper controller to target the position set by start_move().
    // Also updates the limit switch debouncers. Automatically stops when the
    // move is complete or a limit switch is hit.
    void update() {
        switch_top.update();
        switch_bottom.update();
        if (moved_into_top_limit() || moved_into_bottom_limit()) {
            // Cancel the move to prevent a stall
            stepper_.stop_move();
        }
        if (stepper_.remaining_displacement() == 0) {
            // Record that the move has finished
            stepper_.stop_move();
        }
        stepper_.update();
    }

    // Start a move by a given displacement at a given speed.
    // A positive displacement moves the motor up, a negative velocity moves it
    // down. Speed should always be positive. Returns instantly.
    void start_move(float displacement, float target_speed) {
        stepper_.start_move(displacement, target_speed);
    }
    // Start a move at the given velocity. A positive velocity moves the
    // motor up, a negative velocity moves it down. Returns instantly.
    void start_move(float target_velocity) {
        float direction = target_velocity >= 0 ? 1 : -1;
        float target_speed =
            target_velocity >= 0 ? target_velocity : -1 * target_velocity;
        stepper_.start_move(direction * indefinite_displacement, target_speed);
    }
    // Return whether the motion controller is in the middle of a move
    bool moving() { return stepper_.moving(); }

    // Return whether the top limit switch interrupted the most recent move
    bool moved_into_top_limit() {
        using State = DebouncedSwitch::State;

        State top_state = switch_top.getState();
        return stepper_.remaining_displacement() > 0 &&
               (top_state == State::active || top_state == State::activated);
    }
    // Return whether the bottom limit switch interrupted the most recent move
    bool moved_into_bottom_limit() {
        using State = DebouncedSwitch::State;

        State bottom_state = switch_bottom.getState();
        return stepper_.remaining_displacement() < 0 &&
               (bottom_state == State::active ||
                bottom_state == State::activated);
    }

   private:
    // This is greater than the max travel of the linear actuator, so it should
    // hit a limit switch first.
    static constexpr float indefinite_displacement = 300;  // mm
    StepperMotor stepper_;
};

class Door {
   public:
    Door(uint8_t solenoid, uint8_t lock_switch)
        : solenoid_(solenoid, true),
          lock_switch_(
              lock_switch,
              true /* TODO: is switch low when pressed? */,
              true /* TODO: pullup input? */) {}

    // Returns false if the door is open
    bool setup() {
        if (!solenoid_.setup()) {
            // Should never happen
            return false;
        }

        if (!lock_switch_.setup()) {
            // Should never happen
            return false;
        }

        if (!is_open()) {
            start_lock();
            // TODO: do we actually need to try to lock the door as part of
            // setup? Shouldn't the door naturally be locked by default?
            return true;
        } else {
            return false;
        }
    }

    // Update the limit switch debouncer
    void update() { lock_switch_.update(); }

    // Make the solenoid lock. Returns immediately.
    void start_lock() { solenoid_.deactivate(); }
    // Make the solenoid unlock. Returns immediately.
    void start_unlock() { solenoid_.activate(); }
    // Check whether the door is open.
    bool is_open() {
        using State = DebouncedSwitch::State;

        State state = lock_switch_.read();
        return state == State::inactive || state == State::deactivated;
    }

   private:
    DigitalOutput solenoid_;  // by default (when there's no power) the door is
                              // locked; to unlock, write HIGH (need to check)
    DebouncedSwitch lock_switch_;
};