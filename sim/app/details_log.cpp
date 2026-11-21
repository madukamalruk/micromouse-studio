#include "sim/app/details_log.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "sim/app/attitude.hpp"
#include "sim/app/ui_logic.hpp"

namespace mmsim::app {

namespace {

bool contains_ci(std::string_view hay, std::string_view needle) {
    if (needle.empty()) return true;
    if (needle.size() > hay.size()) return false;
    for (std::size_t i = 0; i + needle.size() <= hay.size(); ++i) {
        std::size_t k = 0;
        while (k < needle.size() && std::tolower(static_cast<unsigned char>(hay[i + k])) ==
                                        std::tolower(static_cast<unsigned char>(needle[k])))
            ++k;
        if (k == needle.size()) return true;
    }
    return false;
}

std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* f, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

constexpr double kHalfPi = kPiD / 2.0;
constexpr double kSettleRad = 5.0 * kPiD / 180.0;  // "on an axis" = within 5 degrees
constexpr double kSettleS = 0.015;                 // ... for at least 15 ms
constexpr double kSettleRate = 1.0;                // ... and turning slower than 1 rad/s
constexpr double kStillStepM = 2e-6;               // < 2 mm/s at 1 kHz counts as standing still
constexpr double kStillS = 0.12;

}  // namespace

const char* detail_tag(const DetailEntry& e) {
    if (e.flags & kDetError) return "ERR ";
    if (e.flags & kDetMilestone) return "RUN ";
    if (e.flags & kDetMove) return "MOVE";
    return "FW  ";
}

std::string format_detail(const DetailEntry& e) {
    return fmt("%9.3f  %s  %s", e.t_s, detail_tag(e), e.text.c_str());
}

bool detail_matches(const DetailEntry& e, DetailFilter filter, std::string_view search) {
    switch (filter) {
        case DetailFilter::All:
            break;
        case DetailFilter::Moves:
            if (!(e.flags & kDetMove)) return false;
            break;
        case DetailFilter::Firmware:
            if (!(e.flags & kDetFirmware)) return false;
            break;
        case DetailFilter::Errors:
            if (!(e.flags & kDetError)) return false;
            break;
    }
    return contains_ci(e.text, search) || contains_ci(detail_tag(e), search);
}

// --- MoveTracker -------------------------------------------------------------------------------

void MoveTracker::reset() {
    started_ = false;
    last_t_ = -1.0;
    near_since_ = still_since_ = -1.0;
    run_cells_ = 0;
    history_.clear();
}

std::pair<int, int> MoveTracker::cell_at(double t_s) const {
    if (history_.empty()) return {0, 0};
    std::pair<int, int> c{history_.front().x, history_.front().y};
    for (const auto& h : history_) {
        if (h.t > t_s) break;
        c = {h.x, h.y};
    }
    return c;
}

void MoveTracker::flush_straight(double t, std::vector<DetailEntry>& out) {
    if (run_cells_ >= 2)
        out.push_back({std::max(t, run_last_t_), kDetMove,
                       fmt("Straight %d cells to (%d,%d)", run_cells_, cx_, cy_)});
    run_cells_ = 0;
}

void MoveTracker::feed(const PoseSample& s, std::vector<DetailEntry>& out) {
    const int nx = static_cast<int>(std::floor(s.x / cell_));
    const int ny = static_cast<int>(std::floor(s.y / cell_));
    if (!started_) {
        started_ = true;
        px_ = s.x;
        py_ = s.y;
        theta_u_ = prev_theta_ = s.theta;
        ref_q_ = static_cast<int>(std::lround(theta_u_ / kHalfPi));
        cx_ = nx;
        cy_ = ny;
        last_t_ = s.t_s;
        history_.push_back({s.t_s, cx_, cy_});
        out.push_back({s.t_s, kDetMove,
                       fmt("Start (%d,%d) facing %s", cx_, cy_,
                           quarter_name(quarter_of_bearing(bearing_deg(theta_u_))))});
        return;
    }
    // Unwrapped heading, so a turn through +-180 degrees is one continuous sweep.
    double d = s.theta - prev_theta_;
    d = std::remainder(d, 2.0 * kPiD);
    theta_u_ += d;
    prev_theta_ = s.theta;
    const double dt = s.t_s - last_t_;
    const double rate = dt > 0.0 ? std::fabs(d) / dt : 0.0;
    last_t_ = s.t_s;

    const double step = std::hypot(s.x - px_, s.y - py_);
    px_ = s.x;
    py_ = s.y;

    // Cell transitions.
    if (nx != cx_ || ny != cy_) {
        cx_ = nx;
        cy_ = ny;
        ++run_cells_;
        run_last_t_ = s.t_s;
        history_.push_back({s.t_s, cx_, cy_});
        if (history_.size() > 4096) history_.erase(history_.begin(), history_.begin() + 1024);
        out.push_back({s.t_s, kDetMove,
                       fmt("Cell (%d,%d) %s", cx_, cy_,
                           quarter_name(quarter_of_bearing(bearing_deg(theta_u_))))});
    }

    // Standing still after a run: close the straight.
    if (step < kStillStepM) {
        if (still_since_ < 0.0) still_since_ = s.t_s;
        if (s.t_s - still_since_ >= kStillS && run_cells_ > 0) flush_straight(still_since_, out);
    } else {
        still_since_ = -1.0;
    }

    // Turns: the heading settles on an axis that differs from the last settled one.
    const long q = std::lround(theta_u_ / kHalfPi);
    const double err = std::fabs(theta_u_ - static_cast<double>(q) * kHalfPi);
    // Sweeping through an axis on the way to another one is not a stop: wait for the rate to die.
    if (err < kSettleRad && rate < kSettleRate) {
        if (near_since_ < 0.0) near_since_ = s.t_s;
        if (s.t_s - near_since_ >= kSettleS && q != ref_q_) {
            const int delta4 = static_cast<int>((((q - ref_q_) % 4) + 4) % 4);
            flush_straight(s.t_s, out);
            if (delta4 == 1)
                out.push_back({s.t_s, kDetMove, fmt("Turn Left 90\xC2\xB0 at (%d,%d)", cx_, cy_)});
            else if (delta4 == 3)
                out.push_back({s.t_s, kDetMove, fmt("Turn Right 90\xC2\xB0 at (%d,%d)", cx_, cy_)});
            else if (delta4 == 2)
                out.push_back({s.t_s, kDetMove, fmt("U-turn 180\xC2\xB0 at (%d,%d)", cx_, cy_)});
            ref_q_ = static_cast<int>(q);
        }
    } else {
        near_since_ = -1.0;
    }
}

// --- DetailsLog --------------------------------------------------------------------------------

void DetailsLog::add(DetailEntry e) {
    entries_.push_back(std::move(e));
    while (entries_.size() > kMax) entries_.pop_front();
}

void DetailsLog::ingest_log(double t_s, const std::string& text, std::vector<DetailEntry>& out) {
    unsigned f = kDetFirmware;
    const auto kind = ui::classify_log(text);
    if (kind == ui::EventKind::Error) f |= kDetError;
    out.push_back({t_s, f, text});
    // Readable milestone lines for the Moves view.
    auto has = [&](const char* needle) { return contains_ci(text, needle); };
    if (kind == ui::EventKind::Error) return;
    if (has("goal reached"))
        out.push_back({t_s, kDetMove | kDetMilestone, "GOAL REACHED"});
    else if (has("start reached"))
        out.push_back({t_s, kDetMove | kDetMilestone, "START REACHED (back at the start)"});
    else if (has("speed run start"))
        out.push_back({t_s, kDetMove | kDetMilestone, "Speed run started"});
    else if (has("speed run done"))
        out.push_back({t_s, kDetMove | kDetMilestone, "Speed run finished"});
}

DetailEntry DetailsLog::crash_entry(double t_s, double speed_mps, int cx, int cy) {
    return {t_s, kDetMove | kDetError,
            fmt("Wall crash at %.2f m/s near (%d,%d)", speed_mps, cx, cy)};
}

std::string DetailsLog::to_text(DetailFilter filter, std::string_view search) const {
    std::string out;
    for (const auto& e : entries_)
        if (detail_matches(e, filter, search)) {
            out += format_detail(e);
            out += '\n';
        }
    return out;
}

}  // namespace mmsim::app
