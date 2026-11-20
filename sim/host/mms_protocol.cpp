#include "sim/host/mms_protocol.hpp"

#include <cstdio>

namespace mmsim::host {

using core::Dir;

void mms_step(int dir, int& dx, int& dy) {
    static constexpr int kDx[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    static constexpr int kDy[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    dx = kDx[dir & 7];
    dy = kDy[dir & 7];
}

namespace {

bool wall_at(const core::Maze& m, int cx, int cy, Dir d) {
    if (!m.in_bounds(cx, cy)) return true;
    return m.has_wall(cx, cy, d);
}

/// mms Maze::isWall(SemiPosition, SemiDirection) for one position.
bool is_wall_here(const core::Maze& m, int x, int y, int dir) {
    const int w2 = 2 * m.width(), h2 = 2 * m.height();
    if (x < 0 || y < 0 || x > w2 || y > h2) return true;
    const bool xo = (x & 1) != 0, yo = (y & 1) != 0;
    if (xo && yo) {                // cell centre
        if (dir & 1) return true;  // aims at a post
        static constexpr Dir kCard[4] = {Dir::N, Dir::E, Dir::S, Dir::W};
        return wall_at(m, x / 2, y / 2, kCard[dir / 2]);
    }
    if (!xo && yo) {  // vertical edge (between two cells side by side)
        switch (dir) {
            case 0:
            case 4:
                return true;
            case 2:
            case 6:
                return false;
            case 1:  // NE
                return x == w2 ? false : wall_at(m, x / 2, y / 2, Dir::N);
            case 3:  // SE
                return x == w2 ? false : wall_at(m, x / 2, y / 2, Dir::S);
            case 7:  // NW
                return x == 0 ? false : wall_at(m, x / 2 - 1, y / 2, Dir::N);
            case 5:  // SW
                return x == 0 ? false : wall_at(m, x / 2 - 1, y / 2, Dir::S);
        }
    }
    if (xo && !yo) {  // horizontal edge (between two cells one above the other)
        switch (dir) {
            case 2:
            case 6:
                return true;
            case 0:
            case 4:
                return false;
            case 1:  // NE
                return y == h2 ? false : wall_at(m, x / 2, y / 2, Dir::E);
            case 7:  // NW
                return y == h2 ? false : wall_at(m, x / 2, y / 2, Dir::W);
            case 3:  // SE
                return y == 0 ? false : wall_at(m, x / 2, y / 2 - 1, Dir::E);
            case 5:  // SW
                return y == 0 ? false : wall_at(m, x / 2, y / 2 - 1, Dir::W);
        }
    }
    return true;  // a post position: never reached by legal moves
}

}  // namespace

bool mms_is_wall(const core::Maze& m, SemiPose p, int half_steps_ahead) {
    int dx = 0, dy = 0;
    mms_step(p.dir, dx, dy);
    for (int i = 0; i <= half_steps_ahead; ++i) {
        if (is_wall_here(m, p.x + i * dx, p.y + i * dy, p.dir)) return true;
    }
    return false;
}

int mms_to_int(std::string_view s, bool& ok) {
    ok = false;
    std::size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) ++i;
    std::size_t e = s.size();
    while (e > i && (s[e - 1] == ' ' || s[e - 1] == '\t')) --e;
    s = s.substr(i, e - i);
    if (s.empty()) return 0;
    bool neg = false;
    std::size_t k = 0;
    if (s[0] == '+' || s[0] == '-') {
        neg = s[0] == '-';
        k = 1;
    }
    if (k == s.size()) return 0;
    long long v = 0;
    for (; k < s.size(); ++k) {
        if (s[k] < '0' || s[k] > '9') return 0;
        v = v * 10 + (s[k] - '0');
        if (v > 2147483648LL) return 0;
    }
    if (neg) v = -v;
    if (v > 2147483647LL || v < -2147483648LL) return 0;
    ok = true;
    return static_cast<int>(v);
}

// --- Stats -----------------------------------------------------------------------------

void MmsStats::reset_all() {
    *this = MmsStats();
}

void MmsStats::start_run() {
    cur_dist_ = 0;
    cur_turns_ = 0;
    cur_eff_ = penalty_;
    total_eff_ += penalty_;
    penalty_ = 0;
    cur_valid_ = true;
    started_ = true;
}

void MmsStats::finish_run() {
    started_ = false;
    const float cur = static_cast<float>(cur_turns_) + cur_eff_;
    const float best =
        solved_ ? static_cast<float>(best_turns_) + best_eff_ : std::numeric_limits<float>::max();
    if (cur < best) {
        best_dist_ = cur_dist_;
        best_turns_ = cur_turns_;
        best_eff_ = cur_eff_;
    }
    solved_ = true;
}

void MmsStats::end_unfinished_run() {
    started_ = false;
}

void MmsStats::penalize_for_reset() {
    penalty_ = kResetPenalty;
}

void MmsStats::add_distance(int half_steps) {
    const float eff = effective(half_steps);
    total_dist_ += half_steps;
    total_eff_ += eff;
    if (started_) {
        cur_dist_ += half_steps;
        cur_eff_ += eff;
    }
}

void MmsStats::add_turn() {
    total_turns_ += 1;
    if (started_) cur_turns_ += 1;
}

float MmsStats::score() const {
    if (!solved_) return 2000.0f;
    return best_eff_ + static_cast<float>(best_turns_) +
           0.1f * (total_eff_ + static_cast<float>(total_turns_));
}

namespace {
constexpr std::string_view kNames[] = {"total-distance",
                                       "total-turns",
                                       "best-run-distance",
                                       "best-run-turns",
                                       "current-run-distance",
                                       "current-run-turns",
                                       "total-effective-distance",
                                       "best-run-effective-distance",
                                       "current-run-effective-distance",
                                       "score"};
std::string fmt_float(float f) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%g", static_cast<double>(f));
    return buf;
}
}  // namespace

bool MmsStats::has(std::string_view name) const {
    for (auto n : kNames)
        if (n == name) return true;
    return false;
}

std::string MmsStats::get(std::string_view n) const {
    if (n == "total-distance") return std::to_string(total_dist_);
    if (n == "total-turns") return std::to_string(total_turns_);
    if (n == "total-effective-distance") return fmt_float(total_eff_);
    if (n == "best-run-distance") return solved_ ? std::to_string(best_dist_) : "-1";
    if (n == "best-run-turns") return solved_ ? std::to_string(best_turns_) : "-1";
    if (n == "best-run-effective-distance") return solved_ ? fmt_float(best_eff_) : "-1";
    if (n == "current-run-distance") return cur_valid_ ? std::to_string(cur_dist_) : "-1";
    if (n == "current-run-turns") return cur_valid_ ? std::to_string(cur_turns_) : "-1";
    if (n == "current-run-effective-distance") return cur_valid_ ? fmt_float(cur_eff_) : "-1";
    if (n == "score") return fmt_float(score());
    return "-1";
}

}  // namespace mmsim::host
