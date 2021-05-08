#pragma once

#include "Subsystems.h"
#include "Util.h"

// The InstantDx class provides access to the various devices in
// the system. It also defines routines which take a long time to
// run (and thus may require certain backgrounds tasks to be run),
// such as initializing the devices, waiting for a button press,
// running the test, waiting for temperature to converge on a target, etc.
class InstantDx {
   public:
    InstantDx()
        : user_interface(51, 53),
          camera(45, 47),
          thermal_controller_bottom(44, A1, A0, 1000 * 10, 1000 * 20),
          thermal_controller_top(42, A2, A0, 1000 * 5, 1000 * 5),
          fan(36, true),
          motion_controller(33, 35, 31, 22, 23),
          tickler(40, true),
          door(38, 24) {}

    UserInterface user_interface;
    ESP32Camera camera;
    ThermalController
        thermal_controller_bottom;  // cartridge base heater (for 95 deg C)
    ThermalController
        thermal_controller_top;  // cartridge top heater (for 65 deg C)
    DigitalOutput fan;  // always on, to have good airflow exchange with system
                        // and reduce the system's heat capacity
    MotionController motion_controller;
    DigitalOutput tickler;
    Door door;

    // Initialize everything. Return value describes whether everything
    // succesfully initialized.
    bool await_initialize() {
        SerialUSB.println("  InstantDx::await_initialize: starting...");
        if (!user_interface.setup()) {
            SerialUSB.println("  InstantDx::await_initialize: UI failed!");
            return false;
        }
        user_interface.print_message("Initializing...");
        user_interface.label_buttons();
        /*if (!camera.setup()) {
            SerialUSB.println("  InstantDx::await_initialize: camera failed!");
            return false;
        }*/

        if (!thermal_controller_bottom.setup()) {
            SerialUSB.println(
                "  InstantDx::await_initialize: bottom thermal controller "
                "failed!");
            return false;
        }

        if (!thermal_controller_top.setup()) {
            SerialUSB.println(
                "  InstantDx::await_initialize: top thermal controller "
                "failed!");
            return false;
        }

        if (!fan.setup()) {
            SerialUSB.println("  InstantDx::await_initialize: fan failed!");
            return false;
        }

        fan.activate();
        /*if (!motion_controller.setup()) {
            SerialUSB.println("  InstantDx::await_initialize: motion controller
        failed!"); return false;
        }*/

        if (!tickler.setup()) {
            SerialUSB.println("  InstantDx::await_initialize: tickler failed!");
            return false;
        }

        /*if (!door.setup()) {
            SerialUSB.println("  InstantDx::await_initialize: door failed!");
            return false;
        }*/

        // Let buttons stabilize
        await_sleep(100);
        SerialUSB.println("  InstantDx::await_initialize: completed!");
        return true;
    }

    // Wait for the specified time to elapse. Like the Arduino delay function,
    // but performs background tasks while waiting.
    void await_sleep(unsigned long timeout) {
        SerialUSB.print("  InstantDx::await_sleep(");
        SerialUSB.print(timeout);
        SerialUSB.println(" ms): started!");

        const unsigned long start_time = millis();
        last_thermal_log = start_time;
        // Background_update always runs at least once, even if timeout is 0
        while (true) {
            background_update();
            if (past_timeout(start_time, timeout)) {
                SerialUSB.println("  InstantDx::await_sleep(...): completed!");
                return;
            }
            log_thermal_control("await_sleep(...)");
        }
    }

    // Wait for the specified button to be pressed and then released.
    void await_press(DebouncedSwitch &button) {
        using Event = DebouncedSwitch::State;

        SerialUSB.print("  InstantDx::await_press(");
        if (user_interface.primary.pin == button.pin) {
            SerialUSB.print("primary");
        } else if (user_interface.secondary.pin == button.pin) {
            SerialUSB.print("secondary");
        } else {
            SerialUSB.print(button.pin);
        }
        SerialUSB.println("): started!");

        // Discard previous press/release events
        button.read();
        while (true) {
            background_update();  // updates button debouncing

            if (button.read() == Event::activated) {
                SerialUSB.println("  InstantDx.await_press(...): completed!");
                return;
            }
        }
    }
    // Wait for a single button to be pressed. Return value describes which
    // button was pressed - either primary or secondary. Will not return
    // both/neither; if both were simultaneously pressed, primary button has
    // priority.
    UserInterface::ButtonsState await_press() {
        using Event = DebouncedSwitch::State;
        using ButtonsState = UserInterface::ButtonsState;

        SerialUSB.println("  InstantDx::await_press: started!");

        // Discard previous press/release events
        user_interface.primary.read();
        user_interface.secondary.read();
        while (true) {
            background_update();  // updates button debouncing
            if (user_interface.primary.read() == Event::activated) {
                SerialUSB.println("  InstantDx.await_press: primary pressed!");
                return ButtonsState::primary;
            }
            if (user_interface.secondary.read() == Event::activated) {
                SerialUSB.println(
                    "  InstantDx.await_press: secondary pressed!");
                return ButtonsState::secondary;
            }
        }
    }

    // Wait for heater to warm up past the threshold
    bool await_thermal_warmup(
        ThermalController &thermal_controller,
        float threshold,
        unsigned long timeout) {
        SerialUSB.print("  InstantDx::await_thermal_warmup(");
        if (thermal_controller.pin == thermal_controller_bottom.pin) {
            SerialUSB.print("bottom");
        } else if (thermal_controller.pin == thermal_controller_top.pin) {
            SerialUSB.print("top");
        } else {
            SerialUSB.print(thermal_controller.pin);
        }
        SerialUSB.print(", ");
        SerialUSB.print(threshold);
        SerialUSB.print(" deg C, ");
        SerialUSB.print(timeout);
        SerialUSB.println(" ms): started!");

        thermal_controller.start_control(threshold);
        const unsigned long start_time = millis();
        last_thermal_log = start_time - thermal_log_interval - 1;
        while (true) {
            background_update();  // updates thermal control loop
            log_thermal_control("await_thermal_cooldown(...)");
            if (thermal_controller.thermistor.temperature() > threshold) {
                SerialUSB.println(
                    "  InstantDx.await_thermal_warmup(...): completed!");
                return true;
            }

            if (past_timeout(start_time, timeout)) {
                SerialUSB.println(
                    "  InstantDx.await_thermal_warmup(...): timed out!");
                return false;
            }
        }
    }
    // Wait for both heaters to cool down past the threshold
    bool await_thermal_cooldown(float threshold, unsigned long timeout) {
        SerialUSB.print("  InstantDx::await_thermal_cooldown(");
        SerialUSB.print(threshold);
        SerialUSB.print(" deg C, ");
        SerialUSB.print(timeout);
        SerialUSB.println(" ms): started!");

        thermal_controller_bottom.stop_control();
        thermal_controller_top.stop_control();
        const unsigned long start_time = millis();
        last_thermal_log = start_time - thermal_log_interval - 1;
        while (true) {
            background_update();
            log_thermal_control("await_thermal_cooldown(...)");
            if (thermal_controller_bottom.thermistor.temperature() <
                    threshold &&
                thermal_controller_top.thermistor.temperature() < threshold) {
                SerialUSB.println(
                    "  InstantDx.await_thermal_cooldown(...): completed!");
                return true;
            }

            if (past_timeout(start_time, timeout)) {
                SerialUSB.println(
                    "  InstantDx.await_thermal_cooldown(...): timed out!");
                return false;
            }
        }
    }

    // Unlock the door, wait a while, and assume that it's unlocked
    void await_unlock(unsigned long timeout) {
        SerialUSB.print("  InstantDx::await_unlock(");
        SerialUSB.print(timeout);
        SerialUSB.println(" ms): started!");

        door.start_unlock();
        await_sleep(timeout);  // updates the limit switch debouncer
        // We assume door.is_open() == true without checking.
        SerialUSB.println("  InstantDx::await_unlock(...): completed!");
    }
    // Lock the door, wait a while, and check if it's locked
    bool await_lock(unsigned long timeout) {
        SerialUSB.print("  InstantDx::await_lock(");
        SerialUSB.print(timeout);
        SerialUSB.println(" ms): started!");
        door.start_lock();
        // This approach waits a constant time, to give the solenoid time to
        // fully actuate. This approach assumes that the door is already
        // closed. The timeout would include both solenoid actuation and limit
        // switch bouncing.
        await_sleep(timeout);  // updates the limit switch debouncer
        if (door.is_open()) {
            SerialUSB.println("  InstantDx::await_lock(...): failed!");
        } else {
            SerialUSB.println("  InstantDx::await_lock(...): completed!");
        }
        return !door.is_open();
    }

    // Wait for the specified displacement movement to complete (successfully or
    // unsuccessfully), but give up as a failure if it's not finished within a
    // timeout, or if it was cancelled by a limit switch (if specified).
    bool await_move(
        float displacement, float target_speed, unsigned long timeout) {
        SerialUSB.print("  InstantDx::await_move(");
        SerialUSB.print(displacement);
        SerialUSB.print(" mm, ");
        SerialUSB.print(target_speed);
        SerialUSB.print(" mm/s, ");
        SerialUSB.print(timeout);
        SerialUSB.println(" ms): started!");

        const unsigned long start_time = millis();
        motion_controller.start_move(displacement, target_speed);
        // background_update always runs at least once.
        while (true) {
            background_update();  // updates motion control loop
            if (motion_controller.moved_into_top_limit() ||
                motion_controller.moved_into_bottom_limit()) {
                SerialUSB.print(
                    "  InstantDx::await_move(...): interrupted by ");
                if (motion_controller.moved_into_top_limit() &&
                    motion_controller.moved_into_bottom_limit()) {
                    SerialUSB.print("both limits");
                } else if (motion_controller.moved_into_top_limit()) {
                    SerialUSB.print("top limit");
                } else if (motion_controller.moved_into_bottom_limit()) {
                    SerialUSB.print("bottom limit");
                }
                SerialUSB.println("!");
                return false;
            }

            if (!motion_controller.moving()) {
                SerialUSB.println("  InstantDx::await_move(...): completed!");
                return true;
            }

            if (past_timeout(start_time, timeout)) {
                SerialUSB.println("  InstantDx::await_move(...): timed out!");
                motion_controller.cancel_move();
                return false;
            }
        }
    }

    // Wait for the specified move to a limit to complete, but give up as a
    // failure if it's not finished within a timeout.
    bool await_move(float target_velocity, unsigned long timeout) {
        SerialUSB.print("  InstantDx::await_move(");
        SerialUSB.print(target_velocity);
        SerialUSB.print(" mm/s, ");
        SerialUSB.print(timeout);
        SerialUSB.println(" ms): started!");

        const DebouncedSwitch &limit = target_velocity >= 0
                                           ? motion_controller.switch_top
                                           : motion_controller.switch_bottom;
        motion_controller.start_move(target_velocity);
        bool status = await_limit_press(limit, timeout);
        if (status) {
            SerialUSB.println("  InstantDx::await_move(...): completed!");
        } else {
            SerialUSB.println(
                "  InstantDx::await_move(...): timed out before limit!");
            motion_controller.cancel_move();
        }
        return status;
    }

    // Wait for the specified limit switch to be pressed, but give up
    // as a failure if it's not pressed within a timeout.
    bool await_limit_press(DebouncedSwitch limit, unsigned long timeout) {
        using State = DebouncedSwitch::State;

        SerialUSB.print("  InstantDx::await_limit_press(");
        if (motion_controller.switch_top.pin == limit.pin) {
            SerialUSB.print("top");
        } else if (motion_controller.switch_bottom.pin == limit.pin) {
            SerialUSB.print("bottom");
        } else {
            SerialUSB.print(limit.pin);
        }
        SerialUSB.print(", ");
        SerialUSB.print(timeout);
        SerialUSB.println(" ms): started!");

        const unsigned long start_time = millis();
        // background_update always runs at least once.
        while (true) {
            background_update();  // updates motion control loop and limit
            State state = limit.read();
            if (state == State::active || state == State::activated) {
                SerialUSB.println(
                    "  InstantDx.await_limit_press(...): completed!");
                return true;
            }

            if (past_timeout(start_time, timeout)) {
                SerialUSB.println(
                    "  InstantDx.await_limit_press(...): timed out!");
                return false;
            }
        }
    }

   private:
    static const unsigned long thermal_log_interval = 5 * 1000;

    unsigned long last_thermal_log = 0;

    // Run one update step for anything which needs to be updated frequently
    // e.g. the thermal controller, motion controller, etc.
    void background_update() {
        thermal_controller_bottom.update();
        thermal_controller_top.update();
        user_interface.update();
        motion_controller.update();
        door.update();
    }

    void log_thermal_control(const char *routine) {
        if (!past_timeout(
                last_thermal_log, thermal_log_interval)) {
            return;
        }

        SerialUSB.print("  InstantDx::");
        SerialUSB.print(routine);
        SerialUSB.print(": thermistors at ");
        SerialUSB.print(thermal_controller_bottom.thermistor.temperature());
        SerialUSB.print(" deg C, ");
        SerialUSB.print(thermal_controller_top.thermistor.temperature());
        SerialUSB.println(" deg C!");
        last_thermal_log = millis();
    }
};
