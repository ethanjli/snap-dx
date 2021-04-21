#include "Subsystems.h"

// The InstantDx class provides access to the various devices in
// the system. It also defines routines which take a long time to
// run (and thus may require certain backgrounds tasks to be run),
// such as initializing the devices, waiting for a button press,
// running the test, waiting for temperature to converge on a target, etc.
class InstantDx {
   public:
    enum class TestResult { detected, not_detected, invalid };

    UserInterface user_interface;
    ColorDetector color_detector;
    EventLogger event_logger;
    ThermalController thermal_controller;
    MotionController motion_controller;
    Mixer mixer;
    Door door;

    // Initialize everything. Return value describes whether everything
    // succeeded
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

        return true;
    }
    void handle_initialization_failure() {
        user_interface.report_initialization_failure();
        event_logger.log_initialization_failure();
        door.unlock();
        thermal_controller.stop_control();
    }
    // Run the test and return the test result
    TestResult await_test() {}
    // Wait for the specified button to be pressed
    void await_button(UserInterface::ButtonsState buttons_state) {
        // background_update always runs at least once. Useful if button
        // debouncing happens in the background.
        while (true) {
            background_update();
            if (user_interface.read_buttons() == buttons_state) {
                return;
            }
        }
    }
    // Wait for a single button to be pressed. Return value describes which
    // button was pressed - either primary or secondary.
    // Will not return ButtonsState::both or ButtonsState::neither.
    UserInterface::ButtonsState await_button() {
        using ButtonsState = UserInterface::ButtonsState;
        // background_update always runs at least once. Useful if button
        // debouncing happens in the background.
        while (true) {
            background_update();
            if (user_interface.read_buttons() == ButtonsState::primary) {
                return ButtonsState::primary;
            }

            if (user_interface.read_buttons() == ButtonsState::secondary) {
                return ButtonsState::secondary;
            }
        }
    }
    // Wait for the temperature to converge at the specified value
    void await_temperature(int temperature) {
        thermal_controller.set_temperature(temperature);
        thermal_controller.start_control();
        // background_update always runs at least once. Useful if temperature
        // control & stabilization happens in the background.
        while (true) {
            background_update();
            if (thermal_controller.temperature_converged(temperature)) {
                return;
            }
        }
    }
    // Wait for the specified time to elapse. Like the Arduino delay function,
    // but performs background tasks while waiting.
    void await_timeout(unsigned long timeout) {
        const unsigned long start_time = millis();
        // background_update always runs at least once, even if timeout is 0
        while (true) {
            background_update();
            if (millis() - start_time > timeout) {
                return;
            }
        }
    }

   private:
    // Run one update step for anything which needs to be updated frequently
    // e.g. the thermal controller, motion controller, etc.
    void background_update() {}
};