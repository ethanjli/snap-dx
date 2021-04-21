#include "Subsystems.h"

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
    TestResult await_test() {}
    void await_button(UserInterface::ButtonsState buttons_state) {}
    UserInterface::ButtonsState await_button() {} // must return one or two, not both/neither
    void await_temperature(int temperature) {}
};