#include "rush_extended/rush_extended.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <thread>

namespace rush {

void RushExtended::plan_path_thread_func() {
    const float PLAN_FREQ = param_.path_plan_freq;
    const int PLAN_INTERVAL = std::floor(1000.0 / PLAN_FREQ); // ms

    while (true) {
        const auto START_TIME = std::chrono::system_clock::now();

        const auto mode = this->nav_mode_.load(std::memory_order_relaxed);
        if (mode == NavMode::IDLE) {
            // do nothing
        } else if (mode == NavMode::NORMAL) {
            // TODO: run the normal path plan
        } else if (mode == NavMode::CHASE) {
            // TODO: run special path plan designed for chasing
        } else {
        }

        std::this_thread::sleep_until(START_TIME +
                                      std::chrono::duration_cast<std::chrono::milliseconds>(
                                          std::chrono::milliseconds{PLAN_INTERVAL}));
    }
}

} // namespace rush