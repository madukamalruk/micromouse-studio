#include "sim/core/analysis/battery.hpp"

#include <algorithm>
#include <cmath>

namespace mmsim::core::analysis {

namespace {
constexpr std::pair<double, double> kOcv[] = {{0.0, 3.00},  {0.05, 3.40}, {0.10, 3.55},
                                              {0.20, 3.65}, {0.40, 3.75}, {0.60, 3.85},
                                              {0.80, 4.00}, {1.0, 4.20}};
constexpr Loads kLoads{};
constexpr ThermalPart kU17V2{52.0, 25.0}, kU17Rc{80.0, 25.0}, kU18{50.0, 8.0}, kU13{45.0, 10.0};
constexpr double kRdsOn = 0.36;

void advance(double& tj, double& peak, double t_amb, double p, ThermalPart th, double dt) {
    const double tss = t_amb + p * th.theta_ja;
    tj += (tss - tj) * (1 - std::exp(-dt / th.tau_s));
    peak = std::max(peak, tj);
}
}  // namespace

double lipo_ocv(double soc) {
    soc = std::min(1.0, std::max(0.0, soc));
    for (std::size_t i = 1; i < std::size(kOcv); ++i) {
        const auto [s0, v0] = kOcv[i - 1];
        const auto [s1, v1] = kOcv[i];
        if (soc <= s1) return v0 + (v1 - v0) * (soc - s0) / (s1 - s0);
    }
    return kOcv[std::size(kOcv) - 1].second;
}

std::vector<std::pair<double, double>> mission_profile(double explore_s, double run_s, int runs,
                                                       double idle_between_s, double dt) {
    std::vector<std::pair<double, double>> out;
    double t = 0.0;
    while (t < explore_s) {
        const double pulse =
            std::fmod(t, 1.2) < 0.08 ? kLoads.motor_peak_a : kLoads.motor_explore_a;
        out.push_back({t, pulse});
        t += dt;
    }
    for (int r = 0; r < runs; ++r) {
        double end = t + idle_between_s;
        while (t < end) {
            out.push_back({t, 0.0});
            t += dt;
        }
        end = t + run_s;
        while (t < end) {
            out.push_back({t, (end - t) > run_s - 0.4 ? kLoads.motor_peak_a : kLoads.motor_run_a});
            t += dt;
        }
    }
    return out;
}

BatteryRun battery_run(const std::vector<std::pair<double, double>>& profile, double cap_mah,
                       double r_int, double soc0) {
    BatteryRun b;
    double soc = soc0;
    for (std::size_t k = 0; k < profile.size(); ++k) {
        const double t = profile[k].first;
        const double dt =
            k + 1 < profile.size() ? profile[k + 1].first - t : profile[1].first - profile[0].first;
        const double i = profile[k].second + kLoads.electronics();
        const double v = 2 * lipo_ocv(soc) - r_int * i;
        b.rows.push_back({t, soc, v, i});
        soc -= i * dt / 3600.0 / (cap_mah / 1000.0);
    }
    b.min_v = b.rows.empty() ? 0.0 : b.rows[0].v;
    for (const auto& r : b.rows) b.min_v = std::min(b.min_v, r.v);
    b.end_soc = soc;
    b.mah_used = (soc0 - soc) * cap_mah;
    b.margin_to_6v0 = b.min_v - 6.0;
    return b;
}

Powers powers(double v_bat, double i_motor_total) {
    const double i3 = kLoads.i_3v3_a, i5 = kLoads.i_5v_own_a;
    const double i_each = i_motor_total / 2.0;
    return {std::max(0.0, v_bat - 5.0) * (i5 + i3), (5.0 - 3.3) * i3,
            2 * i_each * i_each * kRdsOn * 1.35};
}

Junctions junction_temps(const BatteryRun& b, double t_amb, bool u17_v2) {
    const ThermalPart u17 = u17_v2 ? kU17V2 : kU17Rc;
    double tj[3] = {t_amb, t_amb, t_amb}, peak[3] = {t_amb, t_amb, t_amb};
    for (std::size_t k = 0; k + 1 < b.rows.size(); ++k) {
        const BatteryRow& r = b.rows[k];
        const double dt = b.rows[k + 1].t - r.t;
        const double im = r.i - kLoads.i_3v3_a - kLoads.i_5v_own_a;
        const Powers p = powers(r.v + 0.15 * r.i, im);
        advance(tj[0], peak[0], t_amb, p.u17, u17, dt);
        advance(tj[1], peak[1], t_amb, p.u18, kU18, dt);
        advance(tj[2], peak[2], t_amb, p.u13, kU13, dt);
    }
    Junctions j;
    j.p_full = powers(8.4, kLoads.motor_run_a);
    j.steady_u17 = t_amb + j.p_full.u17 * u17.theta_ja;
    j.steady_u18 = t_amb + j.p_full.u18 * kU18.theta_ja;
    j.steady_u13 = t_amb + j.p_full.u13 * kU13.theta_ja;
    j.peak_u17 = peak[0];
    j.peak_u18 = peak[1];
    j.peak_u13 = peak[2];
    return j;
}

void ThermalEstimator::reset(double t_amb) {
    amb_ = t_amb;
    for (int i = 0; i < 3; ++i) tj_[i] = peak_[i] = t_amb;
}

void ThermalEstimator::update(double dt, double v_bat, double i_motor_total) {
    const Powers p = powers(v_bat, i_motor_total);
    advance(tj_[0], peak_[0], amb_, p.u17, kU17V2, dt);
    advance(tj_[1], peak_[1], amb_, p.u18, kU18, dt);
    advance(tj_[2], peak_[2], amb_, p.u13, kU13, dt);
}

double ThermalEstimator::peak_max() const {
    return std::max({peak_[0], peak_[1], peak_[2]});
}

AxleLoads axle_loads(double a_mms2, const Chassis& ch) {
    const double lr = ch.wheelbase_mm - ch.cg_from_front_mm;
    const double a = a_mms2 / 1000.0;
    const double sf = ch.mass_kg * kGms2 * lr / ch.wheelbase_mm;
    const double sr = ch.mass_kg * kGms2 * ch.cg_from_front_mm / ch.wheelbase_mm;
    const double shift = ch.mass_kg * a * ch.cg_height_mm / ch.wheelbase_mm;
    return {sf - shift, sr + shift};
}

PitchLimits pitch_limits(const Chassis& ch) {
    return {kGms2 * ch.cg_from_front_mm / ch.cg_height_mm,
            kGms2 * (ch.wheelbase_mm - ch.cg_from_front_mm) / ch.cg_height_mm};
}

}  // namespace mmsim::core::analysis
