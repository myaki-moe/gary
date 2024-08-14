#pragma once

#include <chrono>

using namespace std::chrono;

namespace utils {
    class OfflineDetector {
    public:
        explicit OfflineDetector(double duration);
        void config(double duration);
        void update(bool state);
        bool offline;
    private:
        time_point<system_clock, nanoseconds> last_time;
        double _duration;
        bool flag_update;
    };
}