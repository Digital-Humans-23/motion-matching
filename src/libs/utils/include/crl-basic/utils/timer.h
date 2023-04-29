#pragma once

#include <chrono>
#include <thread>

namespace crl {

class Timer {
#ifdef WIN32
    typedef std::chrono::steady_clock clock;
#elif __APPLE__
    typedef std::chrono::steady_clock clock;
#else
    typedef std::chrono::system_clock clock;
#endif

public:
    Timer() {
        restart();
    }
    ~Timer() {}

    void restart() {
        begin = std::chrono::high_resolution_clock::now();
    }

    double timeEllapsed() {
        now = std::chrono::high_resolution_clock::now();
        duration = now - begin;
        return duration.count();
    }

    static void wait(double t, bool waitAccurately = true) {
        if (waitAccurately) {
            std::chrono::time_point<clock> current = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration_tmp = current - current;
            while (duration_tmp.count() < t) {
                duration_tmp = std::chrono::high_resolution_clock::now() - current;
#ifndef WIN32
                std::this_thread::sleep_for(std::chrono::duration(std::chrono::milliseconds(1))); //Sleep for a bit (too inaccurate to be used on windows)
#endif  // WIN32
            }
        } else {
            std::this_thread::sleep_for(std::chrono::duration(std::chrono::milliseconds((long long)(t * 1000.0))));
        }
    }

private:
    std::chrono::time_point<clock> begin, now;
    std::chrono::duration<double> duration;
};

}  // namespace crl