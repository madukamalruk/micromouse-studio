#include "sim/host/pacer.hpp"

#include <algorithm>
#include <cmath>

namespace mmsim::host {

void Pacer::set_speed(double speed) {
    speed_ = std::max(0.0, speed);
    reset();
}

void Pacer::reset() {
    acc_ms_ = 0.0;
    behind_ = false;
}

std::int64_t Pacer::ticks_for_frame(double wall_dt_s) {
    if (is_max() || !(wall_dt_s > 0.0)) return 0;
    behind_ = wall_dt_s > kMaxCatchUpS;
    const double dt = std::min(wall_dt_s, kMaxCatchUpS);
    acc_ms_ += dt * 1000.0 * speed_;
    const double whole = std::floor(acc_ms_);
    acc_ms_ -= whole;
    return static_cast<std::int64_t>(whole);
}

}  // namespace mmsim::host
