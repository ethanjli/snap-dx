#include <cstdint>  // may not be needed in Arduino IDE

// Hardware components
class LCD {};
class Button {};
class ESP32Camera {};
class Heater {};
class Thermistor {};
class Fan {};
class StepperMotor {};
class LimitSwitch {};
class Tickler {};
class Solenoid {};
class SDCard {};

//  Low-level algorithms

class TemperatureControlLoop {};

// Subsystems

class UserInterface {
   public:
    enum class ButtonsState {
        primary,
        secondary,
        both,
        neither
    };
    bool setup() { }

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

    void set_temperature() {}
    void start_control() {}
    void stop_control() {}
    bool temperature_converged() {}

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

// Integration

class InstantDx {
   public:
    enum class TestResult {
        detected,
        not_detected,
        invalid
    };

    UserInterface user_interface;
    ColorDetector color_detector;
    EventLogger event_logger;
    ThermalController thermal_controller;
    MotionController motion_controller;
    Mixer mixer;
    Door door;

    bool await_initialize() {
        if (!user_interface.setup()) {
            return false;
        }
        user_interface.print_message(/*Initializing. Please wait.*/);
        if (!color_detector.setup()) {
            return false;
        }
        if (!event_logger.setup()) {
            return false;
        }
        if (!thermal_controller.setup()) {
            return false;
        }
        if (!motion_controller.setup()) {
            return false;
        }
        if (!mixer.setup()) {
            return false;
        }
        if (!door.setup()) {
            return false;
        }
    }
    void handle_initialization_failure() {
        user_interface.report_initialization_failure();
        event_logger.log_initialization_failure();
        door.unlock();
        thermal_controller.stop_control();
    }
    TestResult await_test();
    void await_button(UserInterface::ButtonsState buttons_state) {}
    UserInterface::ButtonsState await_button() {} // must return one or two, not both/neither
    void await_temperature(int temperature) {}
};

class Procedure {
   public:
    enum class Step {
        power_on = 0,
        initialize = 1,
        standby = 2,
        load_insert = 3,
        load_close = 4,
        load_lock = 5,
        start = 6,
        run = 7,
        report = 8,
        unload_open = 9,
        unload_clean = 10,
        unload_close = 11,
        unload_done = 12,
        quit = 13,
        power_off = 14
    };

    Step step() { return step_; }
    void go(Step step) { step_ = step; }

    void go_on_button(InstantDx &instant_dx, Step primary, Step secondary) {
        switch (instant_dx.await_button()) {  // guaranteed to return primary/secondary
            case UserInterface::ButtonsState::primary:
                go(primary);
                return;
            case UserInterface::ButtonsState::secondary:
                go(secondary);
                return;
        }
    }

   private:
    Step step_ = Step::power_on;
};

// Global variables are probably not needed

// Main function

void setup() {}  // setup() is probably not needed

void loop() {
    using Step = Procedure::Step;
    using ButtonsState = UserInterface::ButtonsState;
    using TestResult = InstantDx::TestResult;

    InstantDx instant_dx;
    Procedure procedure;
    TestResult test_result;

    switch (procedure.step()) {
        case Step::power_on:
            procedure.go(Step::initialize);
            return;
        case Step::initialize:
            if (!instant_dx.await_initialize()) {
                instant_dx.handle_initialization_failure();  // infinite loop
            }
            return;
        case Step::standby:
            instant_dx.user_interface.print_message(/*InstantDx - v?.?.?*/);
            instant_dx.user_interface.label_buttons(/*new test, --*/);
            instant_dx.await_button(ButtonsState::primary); // new test
            instant_dx.await_temperature(100);  // what should happen when TEMP_!OK?
            procedure.go(Step::load_insert);
            return;
        case Step::load_insert:
            instant_dx.door.unlock();
            // "beep door open alarm", but the schematic has no alarm!
            instant_dx.user_interface.print_message(/*Open and insert*/);
            instant_dx.user_interface.label_buttons(/*done, back*/);
            procedure.go_on_button(instant_dx, Step::load_close, Step::standby);
            return;
        case Step::load_close:
            instant_dx.user_interface.print_message(/*Close and buckle*/);
            instant_dx.user_interface.label_buttons(/*done, back*/);
            procedure.go_on_button(instant_dx, Step::load_lock, Step::load_insert);
            return;
        case Step::load_lock:
            instant_dx.user_interface.print_message(/*Locking door...*/);
            instant_dx.user_interface.label_buttons(); // remove button labels
            instant_dx.door.lock();
            // Do we need to wait a bit to ensure the door's properly closed?
            if (!instant_dx.door.is_open()) {
                procedure.go(Step::start);
                return;
            }
            instant_dx.user_interface.print_message(/*Error. Door not fully closed.*/);
            instant_dx.user_interface.label_buttons(/*done, back*/);
            switch (instant_dx.await_button()) {
                case ButtonsState::primary:
                    // what's supposed to happen if this button is pressed?
                    return;
                case ButtonsState::secondary:
                    procedure.go(Step::load_insert);
                    return;
            }
            return;
        case Step::start:
            instant_dx.user_interface.print_message(/*Press start*/);
            instant_dx.user_interface.label_buttons(/*start, back*/);
            procedure.go_on_button(instant_dx, Step::run, Step::load_insert);
            return;
        case Step::run:
            test_result = instant_dx.await_test();
            // "timer - if limit switch is not detected after # seconds"
            // How is the timer supposed to be used?
            // What's the relationship to the limit switch?
            procedure.go(Step::report);
            return;
        case Step::report:
            instant_dx.user_interface.label_buttons(/*ok, --*/);
            switch (test_result) {
                case TestResult::detected:
                    // Flow chart says to print "un-detected" in the detected case!
                    // I'm pretty sure the flow-chart is wrong...
                    instant_dx.user_interface.print_message(/*Un-detected*/);
                    instant_dx.await_button(ButtonsState::primary);
                    procedure.go(Step::unload_open);
                    return;
                case TestResult::not_detected:
                    // Flow chart says to print "-detected" in the not detected case!
                    // I'm pretty sure the flow-chart is wrong...
                    instant_dx.user_interface.print_message(/*Detected*/);
                    instant_dx.await_button(ButtonsState::primary);
                    procedure.go(Step::unload_open);
                    return;
                case TestResult::invalid:
                    instant_dx.user_interface.print_message(/*Invalid test*/);
                    instant_dx.await_button(ButtonsState::primary);
                    // This implies that we don't unload the sample, clean interior, etc.
                    // Is this what we want?
                    procedure.go(Step::quit);
                    return;
            }
            // What do the TEMP_!OK and TEMP_OK blocks mean in the arrow to 9?
            return;
        case Step::unload_open:
            // what does "wait (1S)" mean here? why is it only on this door operation?
            instant_dx.door.unlock();
            instant_dx.user_interface.print_message(/*Open and unload*/);
            procedure.go_on_button(instant_dx, Step::unload_clean, Step::report);
            return;
        case Step::unload_clean:
            instant_dx.user_interface.print_message(/*Wipe interior*/);
            procedure.go_on_button(instant_dx, Step::unload_close, Step::unload_open);
            return;
        case Step::unload_close:
            instant_dx.user_interface.print_message(/*Close door*/);
            procedure.go_on_button(instant_dx, Step::unload_done, Step::unload_clean);
            return;
        case Step::unload_done:
            instant_dx.door.lock();
            instant_dx.user_interface.print_message(/*Please select:*/);
            procedure.go_on_button(instant_dx, Step::quit, Step::standby);
            return;
        case Step::quit:
            // maybe turn off fan, move stepper to shutdown position, etc.?
            instant_dx.user_interface.print_message(/*Switch off. Goodbye.*/);
            procedure.go(Step::power_off);  // step 14 is handled by the switch's default case
            return;
        case Step::power_off:
            // loop() will do nothing, forever
            return; 
    }
}
