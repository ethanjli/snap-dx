#include "Subsystems.h"

// The InstantDx class provides access to the various devices in
// the system. It also defines routines which take a long time to
// run (and thus may require certain backgrounds tasks to be run),
// such as initializing the devices, waiting for a button press, 
// running the test, waiting for temperature to converge on a target, etc.
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

    // Initialize everything. Return value describes whether everything succeeded
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
    void await_button(UserInterface::ButtonsState buttons_state) {}
    // Wait for a single button to be pressed. Return value describes which button was pressed
    UserInterface::ButtonsState await_button() {} // must return one or two, not both/neither
    // Wait for the temperature to converge at the specified value
    void await_temperature(int temperature) {}
};