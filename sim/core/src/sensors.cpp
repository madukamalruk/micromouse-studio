#include "sim/core/sensors.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <string>

namespace mmsim::core {

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;
constexpr double kGravity = 9.80665;

double quantise(double v, double q) {
    return q > 0.0 ? q * std::round(v / q) : v;
}

std::uint16_t to_u16(double v) {
    if (!(v > 0.0)) return 0;
    if (v >= 65534.0) return 65534;  // 65535 is reserved for "no reading"
    return static_cast<std::uint16_t>(std::lround(v));
}

}  // namespace

void sensor_world_pose(const SensorConfig& s, const Pose& r, Vec2d& origin, Vec2d& dir) {
    const double c = std::cos(r.theta), sn = std::sin(r.theta);
    origin = {r.x + s.x * c - s.y * sn, r.y + s.x * sn + s.y * c};
    const double a = r.theta + s.angle_deg * kDeg;
    dir = {std::cos(a), std::sin(a)};
}

std::uint16_t ir_adc_to_mm(const SensorConfig& s, double adc) {
    const double signal = adc - s.ir_ambient;
    const double floor_signal = s.ir_gain / ((s.range_max + s.ir_d0) * (s.range_max + s.ir_d0));
    if (signal <= 0.0 || signal < floor_signal) return kDistanceNone;
    const double d = std::sqrt(s.ir_gain / signal) - s.ir_d0;
    return to_u16(std::max(0.0, d) * 1000.0);
}

// ---------------------------------------------------------------------------------------------

SensorSuite::SensorSuite(std::vector<SensorConfig> cfg, std::uint64_t world_seed)
    : cfg_(std::move(cfg)), ch_(cfg_.size()) {
    reset(world_seed);
}

void SensorSuite::reset(std::uint64_t world_seed) {
    for (std::size_t i = 0; i < ch_.size(); ++i) {
        Channel& c = ch_[i];
        c = Channel{};
        c.rng.reseed(derive_seed(world_seed, "sensor" + std::to_string(i) + ":" + cfg_[i].name));
    }
}

std::uint16_t SensorSuite::raw(std::size_t ch) const {
    return ch < ch_.size() ? ch_[ch].raw : 0;
}

std::uint16_t SensorSuite::distance_mm(std::size_t ch) const {
    return ch < ch_.size() ? ch_[ch].mm : kDistanceNone;
}

double SensorSuite::true_distance(std::size_t ch) const {
    return ch < ch_.size() ? ch_[ch].true_d : std::numeric_limits<double>::infinity();
}

void SensorSuite::update(std::int64_t now_us, const Pose& robot, const CollisionWorld& world,
                         const std::vector<bool>& emitter_on) {
    for (std::size_t i = 0; i < ch_.size(); ++i) {
        Channel& c = ch_[i];
        const SensorConfig& s = cfg_[i];
        if (now_us >= c.next_sample_us) {
            const std::int64_t period =
                std::max<std::int64_t>(1000, std::llround(1e6 / s.rate_hz / 1000.0) * 1000);
            sample(i, now_us, robot, world, i < emitter_on.size() ? emitter_on[i] : true);
            c.next_sample_us = (c.next_sample_us == 0 ? 0 : c.next_sample_us) + period;
            if (c.next_sample_us <= now_us) c.next_sample_us = now_us + period;
        }
        while (!c.pending.empty() && c.pending.front().ready_us <= now_us) {
            c.raw = c.pending.front().raw;
            c.mm = c.pending.front().mm;
            c.pending.pop_front();
        }
    }
}

void SensorSuite::sample(std::size_t i, std::int64_t now_us, const Pose& robot,
                         const CollisionWorld& world, bool emitter) {
    Channel& c = ch_[i];
    const SensorConfig& s = cfg_[i];
    Vec2d o, centre;
    sensor_world_pose(s, robot, o, centre);
    const double base = std::atan2(centre.y, centre.x);
    const int n = std::max(1, s.rays);
    const double half = 0.5 * s.fov_deg * kDeg;

    c.beam.assign(static_cast<std::size_t>(n), BeamRay{});
    double nearest = std::numeric_limits<double>::infinity();
    double wsum = 0.0, ir_sum = 0.0;
    for (int k = 0; k < n; ++k) {
        const double off = n == 1 ? 0.0 : -half + 2.0 * half * k / (n - 1);
        const double a = base + off;
        const Vec2d d{std::cos(a), std::sin(a)};
        const double weight =
            half > 0.0 ? std::pow(s.edge_weight, (off / half) * (off / half)) : 1.0;
        const RayHit h = world.raycast(o, d, s.range_max);
        BeamRay& br = c.beam[static_cast<std::size_t>(k)];
        br.from = o;
        br.weight = weight;
        br.hit = h.hit;
        const double t = h.hit ? h.t : s.range_max;
        br.to = {o.x + d.x * t, o.y + d.y * t};
        wsum += weight;
        if (h.hit) {
            nearest = std::min(nearest, h.t);
            const double cos_inc = std::max(0.0, -(d.x * h.normal.x + d.y * h.normal.y));
            const double r = std::max(h.t + s.ir_d0, 1e-6);  // 0 when touching a wall, d0 = 0
            ir_sum += weight * s.ir_gain * std::pow(cos_inc, s.ir_incidence_exp) / (r * r);
        }
    }
    // Noise-free centre-ray range (n odd -> exact centre; even -> centre computed separately).
    {
        const RayHit h = world.raycast(o, centre, s.range_max);
        c.true_d = h.hit ? h.t : std::numeric_limits<double>::infinity();
    }

    std::uint16_t raw = 0, mm = kDistanceNone;
    if (s.type == SensorType::ToF) {
        if (std::isfinite(nearest)) {
            double d_mm = std::max(nearest, s.range_min) * 1000.0;
            d_mm += c.rng.normal() * (s.noise_sigma + s.noise_rel * d_mm);
            d_mm = quantise(std::max(0.0, d_mm), s.quant);
            raw = mm = to_u16(d_mm);
        } else {
            (void)c.rng.normal();  // keep the stream aligned whatever the scene
            raw = mm = kDistanceNone;
        }
    } else {
        double adc = s.ir_ambient + (emitter && wsum > 0.0 ? ir_sum / wsum : 0.0);
        adc += c.rng.normal() * (s.noise_sigma + s.noise_rel * adc);
        adc = std::clamp(quantise(adc, s.quant), 0.0, s.ir_adc_max);
        raw = to_u16(adc);
        mm = emitter ? ir_adc_to_mm(s, adc) : kDistanceNone;
    }
    const std::int64_t latency_us = std::llround(s.latency_ms * 1000.0);
    c.pending.push_back({now_us + latency_us, raw, mm});
}

// ---------------------------------------------------------------------------------------------

Imu::Imu(ImuConfig cfg, std::uint64_t world_seed) : cfg_(cfg) {
    reset(world_seed);
}

void Imu::reset(std::uint64_t world_seed) {
    static const char* const g[3] = {"gyro_x", "gyro_y", "gyro_z"};
    static const char* const a[3] = {"accel_x", "accel_y", "accel_z"};
    for (int i = 0; i < 3; ++i) {
        gyro_rng_[i].reseed(derive_seed(world_seed, g[i]));
        accel_rng_[i].reseed(derive_seed(world_seed, a[i]));
    }
    walk_rng_.reseed(derive_seed(world_seed, "gyro_bias_walk"));
    bias_ = cfg_.gyro_bias_dps;
    r_ = ImuReading{};
}

void Imu::update(double dt, double yaw_rate_dps, Vec2d accel_body) {
    bias_ += cfg_.gyro_bias_walk * std::sqrt(dt) * walk_rng_.normal();
    const double true_rate[3] = {0.0, 0.0, yaw_rate_dps};
    for (int i = 0; i < 3; ++i) {
        double g =
            true_rate[i] + (i == 2 ? bias_ : 0.0) + cfg_.gyro_noise_dps * gyro_rng_[i].normal();
        g = std::clamp(quantise(g, cfg_.gyro_quant_dps), -cfg_.gyro_range_dps, cfg_.gyro_range_dps);
        r_.gyro_dps[i] = g;
    }
    const double true_acc[3] = {accel_body.x, accel_body.y, kGravity};
    for (int i = 0; i < 3; ++i) {
        double a = true_acc[i] + cfg_.accel_noise_mps2 * accel_rng_[i].normal();
        a = std::clamp(quantise(a, cfg_.accel_quant_mps2), -cfg_.accel_range_mps2,
                       cfg_.accel_range_mps2);
        r_.accel_mps2[i] = a;
    }
}

}  // namespace mmsim::core
