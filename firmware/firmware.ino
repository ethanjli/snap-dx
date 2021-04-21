#include "InstantDx.h"

// The Procedure class defines a state variable representing which step of the
// procedure we're on. It also provides some utility methods which are used by
// the loop function to set the next step of the procedure to advance to.
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

    // Return which step the procedure is on
    Step step() { return step_; }
    // Change which step the procedure is on
    void go(Step step) { step_ = step; }

    // Wait for a single button-press, then change the step of the procedure
    // based on which button was pressed
    void go_on_button(InstantDx &instant_dx, Step primary, Step secondary) {
        switch (instant_dx.await_button()) {
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

// Global variables

// The InstantDx class is defined in InstantDx.h
InstantDx instant_dx;
// The Procedure class is defined above
Procedure procedure;
// We need to store the test result as a global variable because
// step 9 can jump back to step 8 (which displays the test result),
// so the result needs to persist across steps.
InstantDx::TestResult test_result = InstantDx::TestResult::invalid;

// Main function

// setup() is probably not needed, but this is where we'd set up the Serial
// connection for debugging. Setup of other devices would happen in the setup
// methods for the corresponding devices in HAL.h
void setup() {}

// Each time loop is called, it checks the procedure object to determine the
// current step of the procedure, executes the work defined for that step,
// and then finishes by setting what should be the current step of the procedure
// the next time loop is called (this is defined as part of the work for each
// step). Together, the procedure object and the loop function work as a state
// machine.
void loop() {
    using Step = Procedure::Step;
    using ButtonsState = UserInterface::ButtonsState;
    using TestResult = InstantDx::TestResult;

    // The work done in each step of the procedure is defined in this switch
    // block. Each step of the procedure corresponds to one case statement.
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
            instant_dx.await_button(ButtonsState::primary);  // new test
            instant_dx.await_temperature(
                100);  // what should happen when TEMP_!OK?
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
            procedure.go_on_button(instant_dx, Step::load_lock,
                                   Step::load_insert);
            return;
        case Step::load_lock:
            instant_dx.user_interface.print_message(/*Locking door...*/);
            instant_dx.user_interface.label_buttons();  // remove button labels
            instant_dx.door.lock();
            // Do we need to wait a bit to ensure the door's properly closed?
            if (!instant_dx.door.is_open()) {
                procedure.go(Step::start);
                return;
            }
            instant_dx.user_interface.print_message(
                /*Error. Door not fully closed.*/);
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
                    // Flow chart says to print "un-detected" in the detected
                    // case! I'm pretty sure the flow-chart is wrong...
                    instant_dx.user_interface.print_message(/*Un-detected*/);
                    instant_dx.await_button(ButtonsState::primary);
                    procedure.go(Step::unload_open);
                    return;
                case TestResult::not_detected:
                    // Flow chart says to print "-detected" in the not detected
                    // case! I'm pretty sure the flow-chart is wrong...
                    instant_dx.user_interface.print_message(/*Detected*/);
                    instant_dx.await_button(ButtonsState::primary);
                    procedure.go(Step::unload_open);
                    return;
                case TestResult::invalid:
                    instant_dx.user_interface.print_message(/*Invalid test*/);
                    instant_dx.await_button(ButtonsState::primary);
                    // This implies that we don't unload the sample, clean
                    // interior, etc. Is this what we want?
                    procedure.go(Step::quit);
                    return;
            }
            // What do the TEMP_!OK and TEMP_OK blocks mean in the arrow to 9?
            return;
        case Step::unload_open:
            instant_dx.await_timeout(1000);
            // what does "wait (1S)" mean here? why is it only on this door
            // operation?
            instant_dx.door.unlock();
            instant_dx.user_interface.print_message(/*Open and unload*/);
            procedure.go_on_button(instant_dx, Step::unload_clean,
                                   Step::report);
            return;
        case Step::unload_clean:
            instant_dx.user_interface.print_message(/*Wipe interior*/);
            procedure.go_on_button(instant_dx, Step::unload_close,
                                   Step::unload_open);
            return;
        case Step::unload_close:
            instant_dx.user_interface.print_message(/*Close door*/);
            procedure.go_on_button(instant_dx, Step::unload_done,
                                   Step::unload_clean);
            return;
        case Step::unload_done:
            instant_dx.door.lock();
            instant_dx.user_interface.print_message(/*Please select:*/);
            procedure.go_on_button(instant_dx, Step::quit, Step::standby);
            return;
        case Step::quit:
            // maybe turn off fan, move stepper to shutdown position, etc.?
            instant_dx.user_interface.print_message(/*Switch off. Goodbye.*/);
            procedure.go(Step::power_off);
            return;
        case Step::power_off:
            // loop() will do nothing, forever, because it will be stuck in this
            // state
            return;
    }
}
