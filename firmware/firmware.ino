#include "InstantDx.h"

// The Procedure class defines a state variable representing which step of the
// procedure we're on. It also provides some utility methods which are used by
// the loop function to set the next step of the procedure to advance to.
class Procedure {
   public:
    enum class Step : uint16_t {
        power_on = 0,  // this step is not in the flowchart
        initialize = 1,
        standby = 2,
        load_insert = 3,
        load_close = 4,
        load_lock = 5,
        start = 6,
        // run = 7,  // this step is in the flowchart but is decomposed
        run_heater_1 = 700,   // this step is not in the flowchart
        run_stepper_1 = 701,  // this step is not in the flowchart
        run_stepper_2 = 702,  // this step is not in the flowchart
        run_tickler = 703,    // this step is not in the flowchart
        run_heater_2 = 704,   // this step is not in the flowchart
        run_camera = 705,     // this step is not in the flowchart
        report = 8,
        unload_open = 9,
        unload_clean = 10,
        unload_close = 11,
        unload_lock = 1100,  // this step is not in the flowchart
        unload_done = 12,
        quit = 13,
        maintenance = 14,  // this step is not in the flowchart
        power_off = 15     // this step is not in the flowchart
    };

    struct HeatingParameters {
        float temperature;
        unsigned long warmup_timeout;
        unsigned long hold_duration;
    };

    static const unsigned int door_lock_time = 1000;                  // ms
    static const uint8_t door_safe_temperature = 40;                  // deg C
    static const unsigned long thermal_cooldown_timeout = 60 * 1000;  // ms

    // Return which step the procedure is on.
    Step step() { return step_; }
    // Change which step the procedure is on.
    void go(Step step) {
        step_ = step;
        SerialUSB.print("Procedure.go(");
        SerialUSB.print(static_cast<uint16_t>(step_));
        SerialUSB.println(")");
    }

    // Wait for a single button press-and-release, then change the step of the
    // procedure based on which button was released.
    void go_on_button(
        InstantDx &instant_dx,
        const char *primary_label,
        Step primary,
        const char *secondary_label,
        Step secondary) {
        instant_dx.user_interface.label_buttons(primary_label, secondary_label);
        switch (instant_dx.await_press()) {
            case UserInterface::ButtonsState::primary:
                go(primary);
                return;
            case UserInterface::ButtonsState::secondary:
                go(secondary);
                return;
        }
    }

    // Try to lock the door, then change the step of the procedure based on
    // whether it succeeded.
    void go_try_lock(
        InstantDx &instant_dx, Step success_step, Step retry_step) {
        instant_dx.user_interface.print_message("Locking door...");
        instant_dx.user_interface.label_buttons();  // remove button labels
        if (instant_dx.await_lock(door_lock_time)) {
            go(success_step);
            return;
        }

        instant_dx.user_interface.print_message(
            "Error. Door not fully closed.");
        instant_dx.user_interface.label_buttons("Back");
        instant_dx.await_press(instant_dx.user_interface.primary);
        go(retry_step);
    }

    // Try to cool down the heaters, then change the step of the procedure based
    // on whether it succeeded.
    void go_try_cooldown(
        InstantDx &instant_dx, Step success_step, Step failure_step) {
        if (instant_dx.await_thermal_cooldown(door_safe_temperature, 0)) {
            // Unit is already cool
            go(success_step);
            return;
        }

        instant_dx.user_interface.print_message("Cooling down...");
        if (instant_dx.await_thermal_cooldown(
                door_safe_temperature, thermal_cooldown_timeout)) {
            go(success_step);
            return;
        }

        // Unit could not cool down within the required time
        go(failure_step);
        return;
    }
    // Try to warm up a heater, then change the step of the procedure based
    // on whether it succeeded.
    void go_try_heat(
        InstantDx &instant_dx,
        ThermalController &controller,
        const HeatingParameters &parameters,
        Step success_step,
        Step failure_step) {
        if (!instant_dx.await_thermal_warmup(
                controller,
                parameters.temperature,
                parameters.warmup_timeout)) {
            go(failure_step);
            return;
        }

        instant_dx.await_sleep(parameters.hold_duration);
        controller.stop_control();
        go(success_step);
    }

    // Try to move the stepper by a displacement, then change the step of the
    // procedure based on whether it succeeded.
    void go_try_move(
        InstantDx &instant_dx,
        float displacement,
        float speed,
        unsigned long timeout,
        Step success_step,
        Step failure_step) {
        if (!instant_dx.await_move(displacement, speed, timeout)) {
            go(failure_step);
            return;
        }

        go(success_step);
    }

    // Try to move the stepper to a limit, then change the step of the procedure
    // based on whether it succeeded.
    void go_try_move(
        InstantDx &instant_dx,
        float velocity,
        unsigned long timeout,
        Step success_step,
        Step failure_step) {
        if (!instant_dx.await_move(velocity, timeout)) {
            go(failure_step);
            return;
        }

        go(success_step);
    }

   private:
    Step step_ = Step::power_on;
};

// Global constants

static constexpr Procedure::HeatingParameters thermal_1_heating{
    95.0,            // deg C
    15 * 60 * 1000,  // ms
    3 * 60 * 1000    // ms
};
static constexpr Procedure::HeatingParameters thermal_2_heating{
    65.0,            // deg C
    15 * 60 * 1000,  // ms
    40 * 60 * 1000   // ms
};
static const unsigned int door_unlock_time = 1 * 1000;          // ms
static const unsigned int tickler_duration = 20 * 1000;         // ms
static constexpr float stepper_move_1_displacement = 32;        // mm
static const uint8_t stepper_move_1_speed = 0.5;                // mm/s
static const unsigned long stepper_move_1_timeout = 30 * 1000;  // ms
static const uint8_t stepper_move_2_velocity = 5;               // mm/s
static const unsigned long stepper_move_2_timeout = 5 * 1000;   // ms

// Global variables

// The InstantDx class is defined in InstantDx.h
InstantDx instant_dx;
// The Procedure class is defined above
Procedure procedure;
// We need to store the test result as a global variable because
// step 9 can jump back to step 8 (which displays the test result),
// so the result needs to persist across steps.
ESP32Camera::Result test_result = ESP32Camera::Result::invalid;

// Main function

void setup() {
    SerialUSB.begin(115200);
    while (!SerialUSB) {
        ;  // wait for serial port to connect; needed for native USB port
    }
    SerialUSB.println("DEBUG CONSOLE");
}

// Each time loop is called, it checks the procedure object to determine the
// current step of the procedure, executes the work defined for that step,
// and then finishes by setting what should be the current step of the procedure
// the next time loop is called (this is defined as part of the work for each
// step). Together, the procedure object and the loop function work as a state
// machine.
void loop() {
    using Step = Procedure::Step;
    using ButtonsState = UserInterface::ButtonsState;
    using TestResult = ESP32Camera::Result;

    // The work done in each step of the procedure is defined in this switch
    // block. Each step of the procedure corresponds to one case statement.
    switch (procedure.step()) {
        case Step::power_on:
            procedure.go(Step::initialize);
            return;
        case Step::initialize:
            if (instant_dx.await_initialize()) {
                procedure.go(Step::standby);
                return;
            }

            instant_dx.await_unlock(
                door_unlock_time);  // do we actually want to unlock?
            instant_dx.user_interface.print_message("Initialization failed!");
            instant_dx.user_interface.label_buttons("Ok");
            instant_dx.await_press(instant_dx.user_interface.primary);
            procedure.go(Step::maintenance);
            return;
        case Step::standby:
            instant_dx.user_interface.print_message("InstantDx - v?.?.?");
            instant_dx.user_interface.label_buttons("New Test");
            instant_dx.await_press(instant_dx.user_interface.primary);
            SerialUSB.println("Step::standby: Cooling down...");
            procedure.go_try_cooldown(
                instant_dx, Step::load_insert, Step::maintenance);
            return;
        case Step::load_insert:
            SerialUSB.println("Step::load_insert: Unlocking door...");
            instant_dx.await_unlock(door_unlock_time);
            instant_dx.user_interface.print_message("Open and insert");
            procedure.go_on_button(
                instant_dx, "Done", Step::load_close, "Back", Step::standby);
            return;
        case Step::load_close:
            instant_dx.user_interface.print_message("Close and buckle");
            procedure.go_on_button(
                instant_dx, "Done", Step::load_lock, "Back", Step::load_insert);
            return;
        case Step::load_lock:
            procedure.go_try_lock(instant_dx, Step::start, Step::load_insert);
            return;
        case Step::start:
            instant_dx.user_interface.print_message("Press start");
            procedure.go_on_button(
                instant_dx,
                "Start",
                Step::run_heater_1,
                "Back",
                Step::load_insert);
            return;
        case Step::run_heater_1:
            procedure.go_try_heat(
                instant_dx,
                instant_dx.thermal_controller_1,
                thermal_1_heating,
                Step::run_stepper_1,
                Step::maintenance);
            return;
        case Step::run_stepper_1:
            procedure.go_try_move(
                instant_dx,
                stepper_move_1_displacement,
                stepper_move_1_speed,
                stepper_move_1_timeout,
                Step::run_stepper_2,
                Step::maintenance);
        case Step::run_stepper_2:
            procedure.go_try_move(
                instant_dx,
                stepper_move_2_velocity,
                stepper_move_2_timeout,
                Step::run_tickler,
                Step::maintenance);
        case Step::run_tickler:
            instant_dx.tickler.activate();
            instant_dx.await_sleep(tickler_duration);
            instant_dx.tickler.deactivate();
            procedure.go(Step::run_heater_2);
            return;
        case Step::run_heater_2:
            procedure.go_try_heat(
                instant_dx,
                instant_dx.thermal_controller_2,
                thermal_2_heating,
                Step::run_camera,
                Step::maintenance);
            return;
        case Step::run_camera:
            test_result = instant_dx.camera.read();
            procedure.go(Step::report);
            return;
        case Step::report:
            switch (test_result) {
                case TestResult::positive:
                    instant_dx.user_interface.print_message("Detected");
                    break;
                case TestResult::negative:
                    instant_dx.user_interface.print_message("Not detected");
                    break;
                case TestResult::invalid:
                    // Camera can't tell if it's positive or negative
                    instant_dx.user_interface.print_message("Invalid test");
                    break;
            }
            instant_dx.user_interface.label_buttons("Ok");
            instant_dx.await_press(instant_dx.user_interface.primary);
            procedure.go_try_cooldown(
                instant_dx, Step::unload_open, Step::maintenance);
            return;
        case Step::unload_open:
            instant_dx.await_unlock(door_unlock_time);
            instant_dx.user_interface.print_message("Open and unload");
            procedure.go_on_button(
                instant_dx, "Done", Step::unload_clean, "Back", Step::report);
            return;
        case Step::unload_clean:
            instant_dx.user_interface.print_message("Wipe interior");
            procedure.go_on_button(
                instant_dx,
                "Done",
                Step::unload_close,
                "Back",
                Step::unload_open);
            return;
        case Step::unload_close:
            instant_dx.user_interface.print_message("Close door");
            procedure.go_on_button(
                instant_dx,
                "Done",
                Step::unload_lock,
                "Back",
                Step::unload_clean);
            return;
        case Step::unload_lock:
            procedure.go_try_lock(
                instant_dx, Step::unload_done, Step::unload_open);
            return;
        case Step::unload_done:
            instant_dx.user_interface.print_message("Please select:");
            procedure.go_on_button(
                instant_dx, "End", Step::quit, "New Test", Step::standby);
            return;
        case Step::quit:
            instant_dx.user_interface.print_message("Switch off. Goodbye.");
            procedure.go(Step::power_off);
            return;
        case Step::maintenance:
            instant_dx.thermal_controller_1.stop_control();
            instant_dx.thermal_controller_2.stop_control();
            instant_dx.user_interface.print_message(
                "Hardware error. Switch off. Call for help.");
            procedure.go(Step::power_off);
            return;
        case Step::power_off:
            // loop() will do nothing, forever, because it will be stuck in this
            // state
            return;
    }
}
