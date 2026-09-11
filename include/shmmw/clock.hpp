#pragma once
#include <time.h>
#include <cstdint>

namespace shmmw {

inline int64_t now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// Best-effort sleep to an absolute CLOCK_MONOTONIC deadline. clock_nanosleep
// with TIMER_ABSTIME is used instead of a relative sleep so that per-tick
// drift does not accumulate over a multi-second session.
inline void sleep_until_ns(int64_t deadline_ns) {
    struct timespec ts;
    ts.tv_sec = static_cast<time_t>(deadline_ns / 1000000000LL);
    ts.tv_nsec = static_cast<long>(deadline_ns % 1000000000LL);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
}

} // namespace shmmw
