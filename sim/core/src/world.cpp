#include "sim/core/world.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace mmsim::core {

namespace {

double heading_of(Dir d) {
    switch (d) {
        case Dir::E:
            return 0.0;
        case Dir::N:
            return std::numbers::pi / 2.0;
        case Dir::W:
            return std::numbers::pi;
        case Dir::S:
            return -std::numbers::pi / 2.0;
    }
    return 0.0;
}

Pose lerp(const Pose& a, const Pose& b, double f) {
    return {a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f, a.theta + (b.theta - a.theta) * f};
}

}  // namespace

World::World(Maze maze, RobotConfig cfg, MazeDims dims, std::uint64_t seed)
    : maze_(std::move(maze)),
      cfg_(std::move(cfg)),
      dims_(dims),
      model_(cfg_),
      collide_(maze_, dims_),
      seed_(seed),
      sensors_(cfg_.sensors, seed),
      emitters_(cfg_.sensors.size(), true),
      imu_(cfg_.imu, seed) {
    const double fx = cfg_.body.offset_x + cfg_.body.length / 2.0;
    const double bx = std::abs(cfg_.body.offset_x - cfg_.body.length / 2.0);
    body_radius_ = std::hypot(std::max(fx, bx), cfg_.body.width / 2.0);
    reset();
}

Pose World::start_pose() const {
    const Vec2d c = cell_center(maze_.start(), dims_);
    return {c.x, c.y, heading_of(maze_.start_heading())};
}

void World::reset() {
    s_ = DriveState{};
    s_.pose = start_pose();
    volts_[0] = volts_[1] = 0.0;
    time_us_ = 0;
    crashes_.clear();
    in_contact_ = false;
    model_.reset_counters();
    sensors_.reset(seed_);
    imu_.reset(seed_);
    std::fill(emitters_.begin(), emitters_.end(), true);
    accel_body_ = {};
    accel_sum_ = {};
}

void World::set_motor_duty(int side, double duty) {
    duty = std::clamp(duty, -1.0, 1.0);
    volts_[side & 1] = model_.clamp_voltage(duty * cfg_.battery_voltage);
}

void World::step_tick() {
    const double dt = static_cast<double>(kTickUs) / kSubsteps * 1e-6;
    for (int i = 0; i < kSubsteps; ++i) {
        time_us_ += kTickUs / kSubsteps;
        substep(dt);
    }
    const double tick = static_cast<double>(kTickUs) * 1e-6;
    accel_body_ = {accel_sum_.x / tick, accel_sum_.y / tick};
    accel_sum_ = {};
    sensors_.update(time_us_, s_.pose, collide_, emitters_);
    imu_.update(tick, gyro_z_dps(), accel_body_);
}

namespace {
Vec2d world_velocity(const DriveState& s) {
    const double phi = s.pose.theta - s.beta;
    return {s.v * std::cos(phi), s.v * std::sin(phi)};
}
}  // namespace

void World::accumulate_accel(const DriveState& prev) {
    const Vec2d v0 = world_velocity(prev), v1 = world_velocity(s_);
    const double dvx = v1.x - v0.x, dvy = v1.y - v0.y;
    const double th = 0.5 * (prev.pose.theta + s_.pose.theta);
    const double c = std::cos(th), sn = std::sin(th);
    accel_sum_.x += c * dvx + sn * dvy;
    accel_sum_.y += -sn * dvx + c * dvy;
}

// Continuous collision: sample the swept motion at <= half a wall thickness, then bisect the
// first overlapping interval (12 halvings) and stop at the last free pose. The response
// zeroes the wheel speeds (restitution 0) or reverses them scaled by the restitution. With
// the slip model the body velocity is what the wall stops; the wheels keep spinning.
void World::substep(double dt) {
    const DriveState prev = s_;
    model_.step(s_, volts_[0], volts_[1], dt);
    struct AccelGuard {
        World& w;
        const DriveState& p;
        ~AccelGuard() { w.accumulate_accel(p); }
    } guard{*this, prev};

    const double sweep = std::hypot(s_.pose.x - prev.pose.x, s_.pose.y - prev.pose.y) +
                         std::abs(s_.pose.theta - prev.pose.theta) * body_radius_;
    const int n = std::max(1, static_cast<int>(std::ceil(sweep / (dims_.wall / 2.0))));

    double lo = 0.0, hi = -1.0;
    Contact hit;
    for (int k = 1; k <= n; ++k) {
        const double f = static_cast<double>(k) / n;
        const Contact c = collide_.query(body_obb(lerp(prev.pose, s_.pose, f), cfg_.body));
        if (c.hit) {
            hi = f;
            hit = c;
            break;
        }
        lo = f;
    }
    if (hi < 0.0) {
        in_contact_ = false;
        return;
    }
    for (int i = 0; i < 12; ++i) {
        const double mid = 0.5 * (lo + hi);
        const Contact c = collide_.query(body_obb(lerp(prev.pose, s_.pose, mid), cfg_.body));
        if (c.hit) {
            hi = mid;
            hit = c;
        } else {
            lo = mid;
        }
    }

    const double impact_speed = std::abs(model_.forward_speed(s_));
    s_.pose = lerp(prev.pose, s_.pose, lo);
    const double e = cfg_.collision_restitution;
    if (model_.slip_enabled()) {
        s_.v = -e * s_.v + 0.0;  // + 0.0 normalises -0.0
        s_.w = -e * s_.w + 0.0;
        s_.beta = 0.0;
    } else {
        for (int i = 0; i < 2; ++i) {
            s_.wheel_angle[i] =
                prev.wheel_angle[i] + (s_.wheel_angle[i] - prev.wheel_angle[i]) * lo;
            s_.wheel_speed[i] = -e * s_.wheel_speed[i] + 0.0;
        }
        s_.v = model_.forward_speed(s_);
        s_.w = model_.yaw_rate(s_);
    }
    if (!in_contact_) crashes_.push_back({time_us_, s_.pose, hit.normal, impact_speed, hit.depth});
    in_contact_ = true;
}

// A quadrature decoder counts edges it has passed: floor, not round (the 1e-9 absorbs
// floating error at exact multiples). Wiring inversion flips the sign.
std::int32_t World::encoder_count(int side) const {
    side &= 1;
    const double counts =
        s_.wheel_angle[side] * cfg_.encoder_counts_per_wheel_rev() / (2.0 * std::numbers::pi);
    std::int64_t n = static_cast<std::int64_t>(std::floor(counts + 1e-9));
    if (side == 0 ? cfg_.encoder.invert_left : cfg_.encoder.invert_right) n = -n;
    return static_cast<std::int32_t>(static_cast<std::uint32_t>(n));
}

std::uint16_t World::sensor_raw(int ch) const {
    return ch >= 0 ? sensors_.raw(static_cast<std::size_t>(ch)) : 0;
}

std::uint16_t World::sensor_distance_mm(int ch) const {
    return ch >= 0 ? sensors_.distance_mm(static_cast<std::size_t>(ch)) : kDistanceNone;
}

void World::set_emitter(int ch, bool on) {
    if (ch >= 0 && static_cast<std::size_t>(ch) < emitters_.size())
        emitters_[static_cast<std::size_t>(ch)] = on;
}

bool World::emitter(int ch) const {
    return ch >= 0 && static_cast<std::size_t>(ch) < emitters_.size() &&
           emitters_[static_cast<std::size_t>(ch)];
}

double World::gyro_z_dps() const {
    return model_.yaw_rate(s_) * 180.0 / std::numbers::pi;
}

}  // namespace mmsim::core
