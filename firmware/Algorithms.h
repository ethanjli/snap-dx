class TemperatureControlLoop {};

class Debouncer {
   public:
    enum class Status { ok = 0, waiting, unstable };

    Debouncer() = default;
    // Return debounced signal from input signal
    Status transform(bool input, uint32_t current_time, bool &output) {
        if (current_time - last_sample_time_ < sampling_period_) {
            return Status::waiting;
        }

        last_sample_time_ = current_time;

        // Update the integrator based on the input signal
        if (!input && integrator_ > 0) {
            integrator_--;
        } else if (input && integrator_ < max_integrator_samples) {
            integrator_++;
        }
        // Update the output based on the integrator
        if (integrator_ == 0) {
            output_ = false;
            last_time_stable_ = current_time;
        } else if (integrator_ >= max_integrator_samples) {
            output_ = true;
            last_time_stable_ = current_time;
            integrator_ = max_integrator_samples;  // defensive code if
                                                   // integrator got corrupted
        }
        // Report switch fault if debounce time exceeds the maximum limit
        if (current_time - last_time_stable_ > debounce_time_limit) {
            return Status::unstable;
        }
        output = output_;

        return Status::ok;
    }

   private:
    static const unsigned long debounce_time_limit = 2000;  // ms
    unsigned long sampling_period_ = 1;  // ms
    unsigned long last_sample_time_ = 0;  // ms
    uint8_t integrator_ = 0;
    unsigned long last_time_stable_ = 0;  // ms
    const uint8_t max_integrator_samples = 100;
    bool output_ = false;
};

class EdgeDetector {
   public:
    enum class State {
        no_edge = 0,  // Button is in stable state
        rising_edge,  // Button triggered on rising edge
        falling_edge  // Button triggered on falling edge
    };

    EdgeDetector() = default;
    // Return edge detection signal from input signal
    void transform(bool input, State &output) {
        if (input != last_state_) {
            last_state_ = input;
            if (input) {
                output = State::rising_edge;
            } else {
                output = State::falling_edge;
            }
        } else {
            output = State::no_edge;
        }
    }

   private:
    bool last_state_ = false;
};