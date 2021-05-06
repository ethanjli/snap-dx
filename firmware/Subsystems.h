#include "Algorithms.h"
#include "HAL.h"

// The Subsystems classes define high-level interfaces for
// functionally-related collections of low-level devices.

class DebouncedSwitch {
   public:
    enum class State {
        bouncing,     // switch is transitioning between active and inactive
        deactivated,  // switch was just deactivated in the last read
        inactive,     // switch was deactivated before the last read
        activated,    // switch was just activated down in the last read
        active        // switch was activated down before the last read
    };

    DebouncedSwitch(uint8_t pin, bool active_low, bool pull_up)
        : button_(pin, active_low, pull_up) {}

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
                state_ = State::bouncing;
                return;
            default:
                return;
        }

        EdgeDetector::State state;
        edge_detector_.transform(debounced, state);
        switch (state) {
            case EdgeDetector::State::no_edge:
                break;
            case EdgeDetector::State::rising_edge:
                state_ = State::activated;
            case EdgeDetector::State::falling_edge:
                state_ = State::deactivated;
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
                state_ = State::active;
                break;
            case State::deactivated:
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
    enum class ButtonsState { primary, secondary, both, neither };

    DebouncedSwitch primary;
    DebouncedSwitch secondary;

    UserInterface(uint8_t primary_button, uint8_t secondary_button)
        : primary(primary_button, true /* TODO: is button low when pressed? */,
                  true /* TODO: pullup input? */),
          secondary(secondary_button,
                    true /* TODO: is button low when pressed? */,
                    true /* TODO: pullup input? */) {}

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
        Serial.print("Message: ");
        Serial.println(message);
    }

    // Clear button labels
    void label_buttons() { Serial.println("No buttons!"); }
    // Set primary label, clear secondary label; takes C-style strings
    void label_buttons(const char *primary) {
        Serial.print("Primary button: ");
        Serial.println(primary);
    }
    // Set primary & secondary labels; takes C-style strings
    void label_buttons(const char *primary, const char *secondary) {
        Serial.print("Primary button: ");
        Serial.print(primary);
        Serial.print("; secondary button: ");
        Serial.println(secondary);
    }

   private:
    LCD display_;
};

class ThermalController {
   public:
    Thermistor thermistor;

    ThermalController(uint8_t heater, uint8_t thermistor_sampler,
                      uint8_t thermistor_reference,
                      unsigned long pulse_on_max_duration,
                      unsigned long pulse_off_duration)
        : heater_(heater, true),
          thermistor(thermistor_sampler, thermistor_reference),
          control_loop_(pulse_on_max_duration, pulse_off_duration) {}

    // Should always return true
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
        if (control_loop_.update(thermistor.temperature())) {
            heater_.activate();
        } else {
            heater_.deactivate();
        }
    }

    // Start driving the heater. Returns immediately.
    void start_control(float setpoint) {
        control_loop_.start_control(setpoint);
    }
    // Stop driving the heater, and turn it off. Returns immediately.
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
    unsigned long last_measurement_time_ = 0;
};

class MotionController {
   public:
    DebouncedSwitch switch_top;
    DebouncedSwitch switch_bottom;

    MotionController(uint8_t dir, uint8_t step, uint8_t en, uint8_t switch_top,
                     uint8_t switch_bottom)
        : stepper_(dir, step, en),
          switch_top(switch_top, true /* TODO: low when pressed? */,
                     true /* TODO: pullup input? */),
          switch_bottom(switch_bottom, true /* TODO: low when pressed? */,
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

            if (millis() - start_time > move_timeout) {
                return false;
            }
        }
    }
    // Update stepper controller to target the position set by start_move().
    // Also updates the limit switch debouncers.
    void update() {
        using State = DebouncedSwitch::State;

        // Automatically stop the motor when a limit switch is pressed, to
        // prevent the motor from being forced to stall for extended durations.
        switch_top.update();
        switch_bottom.update();
        State top_state = switch_top.getState();
        State bottom_state = switch_bottom.getState();
        if (stepper_.remaining_estimated_displacement() > 0 &&
            (top_state == State::active || top_state == State::activated)) {
            stop_move();
            return;
        } else if (stepper_.remaining_estimated_displacement() < 0 &&
                   (bottom_state == State::active ||
                    bottom_state == State::activated)) {
            stop_move();
            return;
        }

        // Run the stepper driver
        stepper_.update();
    }

    // Start a move by a given displacement at a given speed.
    // A positive displacement moves the motor up, a negative velocity moves it
    // down. Speed should always be positive. Returns instantly.
    void start_move(float displacement, float target_speed) {
        float target_velocity =
            target_speed >= 0 ? target_speed : -1 * target_speed;
        float direction = displacement >= 0 ? 1 : -1;
        target_velocity *= direction;
        stepper_.start_move(displacement, target_velocity);
    }
    // Start a move at the given velocity. A positive velocity moves the
    // motor up, a negative velocity moves it down. Returns instantly.
    void start_move(float target_velocity) {
        stepper_.start_move(indefinite_displacement, target_velocity);
    }
    // Send the stepper driver to idle, cancelling any incomplete moves.
    // Returns instantly.
    void stop_move() { stepper_.stop_move(); }

   private:
    static constexpr float indefinite_displacement = 300;  // mm
    StepperMotor stepper_;
};

class Door {
   public:
    Door(uint8_t solenoid, uint8_t lock_switch)
        : solenoid_(solenoid, true),
          lock_switch_(lock_switch,
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