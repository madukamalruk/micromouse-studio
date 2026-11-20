#include "sim/host/mms_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sim/core/maze_geometry.hpp"
#include "sim/core/sensors.hpp"

namespace mmsim::host {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDt = 1e-3;  // one World tick
constexpr std::size_t kHistory = 200;

double wrap(double a) {
    return std::remainder(a, 2.0 * kPi);
}

std::vector<std::string_view> split(std::string_view s) {
    std::vector<std::string_view> t;
    std::size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && s[i] == ' ') ++i;
        std::size_t j = i;
        while (j < s.size() && s[j] != ' ') ++j;
        if (j > i) t.push_back(s.substr(i, j - i));
        i = j;
    }
    return t;
}

bool starts_with(std::string_view s, std::string_view p) {
    return s.size() >= p.size() && s.substr(0, p.size()) == p;
}

constexpr std::string_view kAnnotations[] = {"setWall",    "clearWall",     "setColor",
                                             "clearColor", "clearAllColor", "setText",
                                             "clearText",  "clearAllText"};

struct Query {
    std::string_view name;
    int rel;  // eighths clockwise from the heading
};
constexpr Query kWallQueries[] = {{"wallFront", 0},     {"wallFrontRight", 1}, {"wallRight", 2},
                                  {"wallBackRight", 3}, {"wallBack", 4},       {"wallBackLeft", 5},
                                  {"wallLeft", 6},      {"wallFrontLeft", 7}};

}  // namespace

MmsBridge::MmsBridge(core::World& world, Annotations& ann, MmsOptions opt)
    : world_(world), ann_(ann), opt_(opt) {
    const core::Maze& m = world_.maze();
    pose_ = {2 * m.start().x + 1, 2 * m.start().y + 1, 2 * static_cast<int>(m.start_heading())};
    stats_.reset_all();
}

// --- Input -------------------------------------------------------------------------------

void MmsBridge::feed_line(std::string_view raw) {
    std::string line;
    line.reserve(raw.size());
    for (char c : raw)
        if (c != '\r' && c != '\n') line.push_back(c);
    for (auto p : kAnnotations) {
        if (starts_with(line, p)) {
            handle_annotation(split(line), line);
            return;
        }
    }
    queue_.push_back(std::move(line));
}

void MmsBridge::invalid(std::string_view) {
    ++invalid_;
}

void MmsBridge::handle_annotation(const std::vector<std::string_view>& tok, std::string_view line) {
    const std::string_view name = tok.empty() ? std::string_view{} : tok[0];
    bool okx = false, oky = false;
    auto xy = [&](int& x, int& y) {
        x = mms_to_int(tok[1], okx);
        y = mms_to_int(tok[2], oky);
        return okx && oky;
    };
    int x = 0, y = 0;
    if (name == "setWall" || name == "clearWall") {
        if (tok.size() != 4 || !xy(x, y) || tok[3].size() != 1 ||
            std::string_view("nesw").find(tok[3][0]) == std::string_view::npos)
            return invalid(line);
        if (name == "setWall")
            ann_.set_wall(x, y, tok[3][0], true);
        else
            ann_.clear_wall(x, y, tok[3][0]);
    } else if (name == "setColor") {
        if (tok.size() != 4 || !xy(x, y) || tok[3].size() != 1 ||
            Annotations::kPalette.find(tok[3][0]) == std::string_view::npos)
            return invalid(line);
        ann_.set_color(x, y, tok[3][0]);
    } else if (name == "clearColor") {
        if (tok.size() != 3 || !xy(x, y)) return invalid(line);
        ann_.clear_color(x, y);
    } else if (name == "clearAllColor") {
        if (tok.size() != 1) return invalid(line);
        ann_.clear_all_color();
    } else if (name == "setText") {
        // "setText X Y TEXT": the text is everything after the third space (may hold spaces).
        std::size_t sp[3];
        std::size_t from = 0;
        for (auto& s : sp) {
            s = line.find(' ', from);
            if (s == std::string_view::npos) return invalid(line);
            from = s + 1;
        }
        bool ok1 = false, ok2 = false;
        x = mms_to_int(line.substr(sp[0] + 1, sp[1] - sp[0] - 1), ok1);
        y = mms_to_int(line.substr(sp[1] + 1, sp[2] - sp[1] - 1), ok2);
        if (line.substr(0, sp[0]) != "setText" || !ok1 || !ok2) return invalid(line);
        std::string text(line.substr(sp[2] + 1));
        for (char& c : text)
            if (c < 32 || c > 126) c = '?';
        ann_.set_text(x, y, text);
    } else if (name == "clearText") {
        if (tok.size() != 3 || !xy(x, y)) return invalid(line);
        ann_.set_text(x, y, "");
    } else if (name == "clearAllText") {
        if (tok.size() != 1) return invalid(line);
        ann_.clear_all_text();
    } else {
        invalid(line);
    }
}

void MmsBridge::process() {
    while (!motion_ && !queue_.empty()) {
        std::string line = std::move(queue_.front());
        queue_.pop_front();
        execute(line);
    }
}

void MmsBridge::execute(const std::string& line) {
    const auto tok = split(line);
    if (tok.empty() || tok.size() > 2) return invalid(line);
    const std::string_view name = tok[0];
    const bool has_arg = tok.size() == 2;
    int query_rel = -1;
    for (const auto& q : kWallQueries)
        if (q.name == name) query_rel = q.rel;
    const bool arg_ok =
        query_rel >= 0 || name == "getStat" || name == "moveForward" || name == "moveForwardHalf";
    if (has_arg && !arg_ok) return invalid(line);

    history_.push_back(line);
    if (history_.size() > kHistory) history_.pop_front();

    bool ok = false;
    const int n = has_arg ? mms_to_int(tok[1], ok) : 1;  // mms: no ok check, bad -> 0
    const core::Maze& m = world_.maze();
    if (name == "mazeWidth") return respond(std::to_string(m.width()));
    if (name == "mazeHeight") return respond(std::to_string(m.height()));
    if (query_rel >= 0) return respond(wall_query(query_rel, n - 1) ? "true" : "false");
    if (name == "moveForward") return start_move(2 * n);
    if (name == "moveForwardHalf") return start_move(n);
    if (name == "turnRight" || name == "turnRight90") return start_turn(2);
    if (name == "turnLeft" || name == "turnLeft90") return start_turn(-2);
    if (name == "turnRight45") return start_turn(1);
    if (name == "turnLeft45") return start_turn(-1);
    if (name == "wasReset") return respond(was_reset_ ? "true" : "false");
    if (name == "ackReset") {
        const SemiPose start{2 * m.start().x + 1, 2 * m.start().y + 1,
                             2 * static_cast<int>(m.start_heading())};
        teleport(start);
        pose_ = start;
        stats_.penalize_for_reset();
        stats_.end_unfinished_run();
        was_reset_ = false;
        latched_at_ = {-1, -1, -1};
        return respond("ack");
    }
    if (name == "getStat") {
        if (!has_arg || !stats_.has(tok[1])) {
            history_.pop_back();
            return invalid(line);
        }
        return respond(stats_.get(tok[1]));
    }
    history_.pop_back();
    invalid(line);
}

std::vector<std::string> MmsBridge::take_responses() {
    std::vector<std::string> r;
    r.swap(out_);
    return r;
}

// --- Walls -------------------------------------------------------------------------------

bool MmsBridge::wall_query(int rel, int half_steps) {
    SemiPose p = pose_;
    p.dir = (pose_.dir + rel) & 7;
    if (opt_.walls == MmsWallSource::Sensors) {
        if (auto s = sensor_wall(rel, half_steps)) return *s;
        ++fallbacks_;
    }
    return mms_is_wall(world_.maze(), p, std::max(half_steps, 0));
}

std::optional<bool> MmsBridge::sensor_wall(int rel, int half_steps) {
    const core::DriveState& d = world_.drive();
    const bool centre = (pose_.x & 1) && (pose_.y & 1) && (pose_.dir & 1) == 0;
    if (!centre || half_steps > 0 || std::abs(d.v) > 0.02 || std::abs(d.w) > 0.2)
        return std::nullopt;
    const auto& sens = world_.sensors();
    auto reading = [&](std::size_t ch) {
        const std::uint16_t mm = world_.sensor_distance_mm(static_cast<int>(ch));
        return mm == core::kDistanceNone ? 1e9 : static_cast<double>(mm);
    };
    if (rel == 0) {
        double sum = 0;
        int cnt = 0;
        for (std::size_t ch = 0; ch < sens.size(); ++ch)
            if (std::abs(sens.config(ch).angle_deg) < 20.0) {
                sum += reading(ch);
                ++cnt;
            }
        if (cnt == 0) return std::nullopt;
        return sum / cnt < opt_.front_wall_mm;
    }
    if (rel != 2 && rel != 6) return std::nullopt;
    const bool left = rel == 6;
    // Side-looking sensors see the side wall from the cell centre.
    for (std::size_t ch = 0; ch < sens.size(); ++ch) {
        const double a = sens.config(ch).angle_deg;
        if (std::abs(a) >= 70.0 && (a > 0) == left) return reading(ch) < opt_.side_wall_mm;
    }
    // Diagonal sensors: use what they saw on the way into this cell.
    if (latched_at_ == pose_) {
        const int v = left ? latched_left_ : latched_right_;
        if (v >= 0) return v == 1;
    }
    return std::nullopt;
}

// --- Motion ------------------------------------------------------------------------------

core::Pose MmsBridge::world_pose(const SemiPose& p) const {
    const double half = world_.dims().cell / 2.0;
    return {p.x * half, p.y * half, kPi / 2.0 - p.dir * kPi / 4.0};
}

bool MmsBridge::at_start(const SemiPose& p) const {
    const core::Cell s = world_.maze().start();
    return p.x / 2 == s.x && p.y / 2 == s.y;
}

void MmsBridge::start_move(int half_steps) {
    if (half_steps < 1) {
        ++crashes_;
        return respond("crash");
    }
    const core::Maze& m = world_.maze();
    Motion mo;
    mo.kind = Motion::Straight;
    const core::Pose p0 = world_pose(pose_);
    mo.x0 = p0.x;
    mo.y0 = p0.y;
    mo.th0 = p0.theta;
    mo.crashes_at_start = world_.crashes().size();
    ++moves_;
    if (mms_is_wall(m, pose_, 0)) {
        // Blocked: drive into the wall for real, then answer crash without moving (mms).
        mo.len = 0;
        mo.doomed = true;
        mo.target = pose_;
        mo.reply_crash = true;
        motion_ = mo;
        return;
    }
    int allowable = 1;
    while (allowable < half_steps && !mms_is_wall(m, pose_, allowable)) ++allowable;
    if (at_start(pose_)) stats_.start_run();
    stats_.add_distance(half_steps);
    int dx = 0, dy = 0;
    mms_step(pose_.dir, dx, dy);
    mo.target = {pose_.x + allowable * dx, pose_.y + allowable * dy, pose_.dir};
    mo.len = allowable * world_.dims().cell / 2.0 * ((pose_.dir & 1) ? std::sqrt(2.0) : 1.0);
    mo.doomed = allowable != half_steps;
    mo.reply_crash = mo.doomed;
    mo.latch_sides = opt_.walls == MmsWallSource::Sensors && (mo.target.x & 1) &&
                     (mo.target.y & 1) && (mo.target.dir & 1) == 0;
    latch_left_ = latch_right_ = std::numeric_limits<double>::infinity();
    latched_at_ = {-1, -1, -1};
    motion_ = mo;
}

void MmsBridge::start_turn(int eighths) {
    stats_.add_turn();
    Motion mo;
    mo.kind = Motion::Turn;
    const core::Pose p0 = world_pose(pose_);
    mo.x0 = p0.x;
    mo.y0 = p0.y;
    mo.th0 = p0.theta;
    mo.ang = -eighths * kPi / 4.0;  // clockwise = negative theta
    mo.target = {pose_.x, pose_.y, (pose_.dir + eighths + 8) & 7};
    mo.crashes_at_start = world_.crashes().size();
    ++moves_;
    latched_at_ = {-1, -1, -1};
    motion_ = mo;
}

void MmsBridge::drive_wheels(double v, double w, double a_ff) {
    const core::RobotConfig& c = world_.config();
    const double b = c.track_width, r = c.wheel_radius;
    const double g = c.motor.gear_ratio, kt = c.motor.kt, ke = c.motor.ke, R = c.motor.resistance;
    const double eff = c.motor.efficiency > 0 ? c.motor.efficiency : 1.0;
    const double kv = ke * g / r;                             // V per m/s of wheel speed (back-EMF)
    const double ks = c.motor.coulomb * R / (kt * eff);       // V to break static friction
    const double ka = R * c.mass * r / (2.0 * g * eff * kt);  // V per m/s^2 (half the mass)
    constexpr double kp = 10.0;                               // V per m/s of wheel-speed error
    const double target[2] = {v - w * b / 2.0, v + w * b / 2.0};
    const core::DriveState& s = world_.drive();
    for (int i = 0; i < 2; ++i) {
        const double vt = target[i];
        const double act = s.wheel_speed[i] * r;
        double volts = kv * vt + ka * a_ff + kp * (vt - act);
        if (std::abs(vt) > 1e-4) volts += std::copysign(ks, vt);
        world_.set_motor_duty(i, volts / world_.battery_voltage());
    }
}

void MmsBridge::control() {
    Motion& mo = *motion_;
    const core::DriveState& d = world_.drive();
    const core::Pose& p = d.pose;
    const bool straight = mo.kind == Motion::Straight;
    const double total = straight ? mo.len : std::abs(mo.ang);
    const double amax = straight ? opt_.accel : opt_.alpha;
    const double vmax = straight ? opt_.v_max : opt_.w_max;
    const double v_old = mo.v_ref;
    if (!mo.profile_done) {
        const double rem = total - mo.s_ref;
        mo.v_ref =
            std::min({mo.v_ref + amax * kDt, vmax, std::sqrt(2.0 * amax * std::max(rem, 0.0))});
        if (rem <= mo.v_ref * kDt || rem <= 1e-9) {
            mo.s_ref = total;
            mo.v_ref = 0;
            mo.profile_done = true;
        } else {
            mo.s_ref += mo.v_ref * kDt;
        }
    }
    const double acc = (mo.v_ref - v_old) / kDt;

    if (straight) {
        const double ux = std::cos(mo.th0), uy = std::sin(mo.th0);
        const double rx = p.x - mo.x0, ry = p.y - mo.y0;
        const double along = rx * ux + ry * uy;
        const double lat = ux * ry - uy * rx;  // + = left of the line
        // Sensor mode: watch the diagonals while they look at the target cell's side walls.
        if (mo.latch_sides) {
            const auto& sens = world_.sensors();
            const double half_in = world_.dims().cell / 2.0 - world_.dims().wall / 2.0;
            for (std::size_t ch = 0; ch < sens.size(); ++ch) {
                const auto& sc = sens.config(ch);
                const double a = std::abs(sc.angle_deg) * kPi / 180.0;
                if (a < 20.0 * kPi / 180.0 || a >= 70.0 * kPi / 180.0) continue;
                const double reach = sc.x + (half_in - std::abs(sc.y)) / std::tan(a);
                if (std::abs((mo.len - along) - reach) > 0.010) continue;
                const std::uint16_t mm = world_.sensor_distance_mm(static_cast<int>(ch));
                const double v = mm == core::kDistanceNone ? 1e9 : mm;
                double& slot = sc.angle_deg > 0 ? latch_left_ : latch_right_;
                slot = std::min(slot, v);
            }
        }
        const double eth = wrap(mo.th0 - p.theta);
        if (mo.pushing) {
            drive_wheels(opt_.push_speed, 15.0 * eth, 0.0);
            mo.push_s += opt_.push_speed * kDt;
            if (world_.crashes().size() > mo.crashes_at_start || mo.push_s > 0.3) finish_motion();
            return;
        }
        if (!mo.doomed && world_.crashes().size() > mo.crashes_at_start) {
            // Hit something on a move the maze allows (controller fault): report and recover.
            mo.target = {pose_.x, pose_.y, pose_.dir};
            mo.reply_crash = true;
            finish_motion();
            return;
        }
        const double e_s = mo.s_ref - along;
        const double steer = std::min(1.0, std::abs(d.v) / 0.1);
        const double th_des = mo.th0 - steer * std::atan2(lat, 0.05);
        const double v_cmd = mo.v_ref + 8.0 * e_s;
        const double w_cmd = 15.0 * wrap(th_des - p.theta);
        drive_wheels(v_cmd, w_cmd, acc);
        if (mo.profile_done) {
            if (mo.doomed) {
                mo.pushing = true;
                return;
            }
            const bool settled = std::abs(e_s) < 0.0015 && std::abs(eth) < 0.0087 &&
                                 std::abs(d.v) < 0.03 && std::abs(d.w) < 0.3;
            if (settled || ++mo.settle_ticks > 400) finish_motion();
        }
        return;
    }

    // Turn in place: heading follows the profile, position held at the start point.
    const double sg = mo.ang < 0 ? -1.0 : 1.0;
    const double th_ref = mo.th0 + sg * mo.s_ref;
    const double eth = wrap(th_ref - p.theta);
    const double hx = std::cos(p.theta), hy = std::sin(p.theta);
    const double e_pos = (mo.x0 - p.x) * hx + (mo.y0 - p.y) * hy;
    drive_wheels(8.0 * e_pos, sg * mo.v_ref + 15.0 * eth, 0.0);
    if (mo.profile_done) {
        const bool settled = std::abs(eth) < 0.0087 && std::abs(e_pos) < 0.0015 &&
                             std::abs(d.v) < 0.03 && std::abs(d.w) < 0.3;
        if (settled || ++mo.settle_ticks > 400) finish_motion();
    }
    (void)acc;
}

void MmsBridge::teleport(const SemiPose& sp) {
    core::DriveState& s = world_.drive_mut();
    s.pose = world_pose(sp);
    s.wheel_speed[0] = s.wheel_speed[1] = 0.0;
    s.v = s.w = s.beta = 0.0;
    world_.set_motor_duty(0, 0.0);
    world_.set_motor_duty(1, 0.0);
}

void MmsBridge::finish_motion() {
    Motion mo = *motion_;
    motion_.reset();
    world_.set_motor_duty(0, 0.0);
    world_.set_motor_duty(1, 0.0);
    const bool moved = !(mo.kind == Motion::Straight && mo.len == 0);
    if (mo.reply_crash) teleport(mo.target);
    pose_ = mo.target;
    if (mo.latch_sides && !mo.reply_crash) {
        latched_at_ = pose_;
        latched_left_ = latch_left_ < opt_.side_wall_mm ? 1 : (std::isfinite(latch_left_) ? 0 : -1);
        latched_right_ =
            latch_right_ < opt_.side_wall_mm ? 1 : (std::isfinite(latch_right_) ? 0 : -1);
    }
    if (mo.reply_crash) {
        ++crashes_;
        respond("crash");
    } else {
        respond("ack");
    }
    if (moved) after_movement();
}

void MmsBridge::after_movement() {
    const core::Cell c{pose_.x / 2, pose_.y / 2};
    if (world_.maze().is_goal(c)) {
        if (stats_.run_started()) stats_.finish_run();
    } else if (at_start(pose_)) {
        stats_.end_unfinished_run();
    }
}

void MmsBridge::tick() {
    process();
    if (motion_) {
        control();
    } else {
        world_.set_motor_duty(0, 0.0);
        world_.set_motor_duty(1, 0.0);
    }
    world_.step_tick();
    if (!motion_) process();
}

}  // namespace mmsim::host
