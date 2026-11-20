#include "sim/core/analysis/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <numeric>

#include "sim/core/analysis/trajectory.hpp"

namespace mmsim::core::analysis {

namespace {
std::string fmt(const char* f, double a, double b = 0) {
    char buf[160];
    std::snprintf(buf, sizeof buf, f, a, b);
    return buf;
}
}  // namespace

StepMetrics step_metrics(const std::vector<double>& t, const std::vector<double>& y,
                         double min_step) {
    StepMetrics m;
    const std::size_t n = y.size();
    if (n < 20 || t.size() != n) {
        m.error = "too few samples";
        return m;
    }
    std::size_t peak_i = 0;
    for (std::size_t i = 1; i < n; ++i)
        if (std::abs(y[i]) > std::abs(y[peak_i])) peak_i = i;
    const double sign = y[peak_i] >= 0 ? 1.0 : -1.0;
    std::vector<double> z(n);
    for (std::size_t i = 0; i < n; ++i) z[i] = sign * y[i];
    const double zmax = z[peak_i];
    if (zmax < min_step) {
        m.error = fmt("no step: the largest speed is only %.0f counts/s (need at least %.0f)", zmax,
                      min_step);
        return m;
    }
    std::size_t i10 = 0;
    while (z[i10] < 0.1 * zmax) ++i10;
    auto median_of = [](std::vector<double> v) {
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    };
    const std::size_t nb = std::max<std::size_t>(i10, 1);
    const double base = i10 >= 5 ? median_of({z.begin(), z.begin() + static_cast<long>(nb)}) : z[0];
    std::size_t end = n;
    for (std::size_t i = peak_i; i < n; ++i)
        if (z[i] < 0.5 * zmax) {
            end = i;
            break;
        }
    const std::vector<double> tail(z.begin() + static_cast<long>(peak_i),
                                   z.begin() + static_cast<long>(end));
    const std::size_t k = std::max<std::size_t>(3, tail.size() / 5);
    const bool cut = end >= k && end - k > peak_i;
    const std::vector<double> plateau =
        cut ? std::vector<double>(z.begin() + static_cast<long>(end - k),
                                  z.begin() + static_cast<long>(end))
            : tail;
    const double final_v = median_of(plateau);
    const double amp = final_v - base;
    if (amp < min_step) {
        m.error = "no plateau after the step";
        return m;
    }
    auto cross = [&](double level) -> std::optional<double> {
        for (std::size_t i = 0; i < n; ++i) {
            if (z[i] >= level) {
                if (i == 0) return t[0];
                const double f = z[i] != z[i - 1] ? (level - z[i - 1]) / (z[i] - z[i - 1]) : 0.0;
                return t[i - 1] + f * (t[i] - t[i - 1]);
            }
        }
        return std::nullopt;
    };
    const auto t10 = cross(base + 0.1 * amp), t90 = cross(base + 0.9 * amp);
    if (!t10 || !t90) {
        m.error = "the speed never reached 90 % of the plateau";
        return m;
    }
    const double band = 0.02 * amp;
    std::optional<std::size_t> last_out;
    for (std::size_t i = i10; i < end; ++i)
        if (std::abs(z[i] - final_v) > band) last_out = i;
    const double t_settle = !last_out ? t[0] : t[std::min(*last_out + 1, n - 1)];
    m.rise_time_s = *t90 - *t10;
    m.settling_time_s = std::max(0.0, t_settle - *t10);
    m.overshoot_pct = std::max(0.0, (zmax - final_v) / amp * 100.0);
    m.final_value = sign * final_v;
    m.baseline = sign * base;
    m.peak = sign * zmax;
    m.t10_s = *t10;
    m.t90_s = *t90;
    m.t_start_s = t[i10 > 0 ? i10 - 1 : 0];
    m.samples = static_cast<int>(n);
    return m;
}

double steady_state_error(const StepMetrics& m, double setpoint) {
    return m.ok() ? std::abs(setpoint - m.final_value) : 0.0;
}

ImuStats imu_stats(const std::vector<double>& s) {
    ImuStats r;
    r.n = static_cast<int>(s.size());
    if (s.size() < 2) return r;
    const double mean = std::accumulate(s.begin(), s.end(), 0.0) / static_cast<double>(s.size());
    double var = 0;
    for (const double v : s) var += (v - mean) * (v - mean);
    var /= static_cast<double>(s.size() - 1);
    const auto [mn, mx] = std::minmax_element(s.begin(), s.end());
    r.mean = mean;
    r.stddev = std::sqrt(var);
    r.p2p = *mx - *mn;
    return r;
}

Cornering cornering(double v, double r, double mu) {
    Cornering c;
    if (!std::isfinite(v) || !std::isfinite(r) || !std::isfinite(mu) || v < 0 || r <= 0 ||
        mu <= 0) {
        c.error = "speed must be >= 0, radius and friction > 0";
        return c;
    }
    c.v_mms = v;
    c.radius_mm = r;
    c.mu = mu;
    c.a_lat_mms2 = v * v / r;
    c.a_lat_g = c.a_lat_mms2 / kGmms2;
    c.limit_mms = std::sqrt(mu * kGmms2 * r);
    c.limit_g = mu;
    c.ratio = v / c.limit_mms;
    c.verdict =
        c.ratio <= 0.9 ? "WITHIN TRACTION" : (c.ratio <= 1.0 ? "NEAR LIMIT" : "EXCEEDS TRACTION");
    c.within = c.ratio <= 1.0;
    return c;
}

BatteryHealth battery_health(double rest_mv, double load_mv, double current_a,
                             std::optional<double> peak_a, double reset_mv) {
    BatteryHealth h;
    const bool finite = std::isfinite(rest_mv) && std::isfinite(load_mv) &&
                        std::isfinite(current_a) && std::isfinite(reset_mv) &&
                        (!peak_a || std::isfinite(*peak_a));
    if (!finite) {
        h.error = "rest, load, current and reset must be numbers";
    } else if (current_a <= 0 || (peak_a && *peak_a <= 0)) {
        h.error = "the motor current must be > 0";
    } else if (rest_mv <= 0 || load_mv <= 0 || reset_mv <= 0) {
        h.error = "voltages must be > 0";
    } else if (load_mv > rest_mv) {
        h.error =
            "the loaded voltage is above the resting voltage: read the pack at rest first, then "
            "under load";
    }
    if (!h.error.empty()) return h;
    h.rint_mohm = (rest_mv - load_mv) / current_a;
    h.peak_a = peak_a ? *peak_a : current_a;
    h.sag_mv = h.rint_mohm * h.peak_a;
    h.predicted_min_mv = rest_mv - h.sag_mv;
    h.healthy = h.rint_mohm < kRintHealthyMohm;
    h.margin_mv = h.predicted_min_mv - reset_mv;
    h.brownout_risk = h.margin_mv < 0 || !h.healthy;
    h.badge = h.healthy ? "HEALTHY (< 150 m\xCE\xA9)" : "HIGH IMPEDANCE / BROWNOUT RISK";
    return h;
}

FaultInfo neomouse_fault_info(int code) {
    static const std::map<int, FaultInfo> table = {
        {0, {0, "NONE", "No fault latched.", "Nothing to do."}},
        {1,
         {1, "CLOCK FAILURE", "The 25 MHz crystal / PLL did not start or was lost.",
          "Check the crystal, its load capacitors and 3V3; reflash. The system stays in FAULT "
          "until power-cycled."}},
        {2,
         {2, "WATCHDOG RESET",
          "The previous run ended in a watchdog reset, i.e. the firmware stopped feeding it.",
          "Find what blocked the main loop (a flash erase wait, a hung bus). Hold both buttons for "
          "1 s to clear after fixing."}},
        {3,
         {3, "TICK OVERRUN", "The 1 kHz safety tick took longer than its budget.",
          "Something in the tick is too slow (a build without optimisation, a new feature). "
          "Measure the tick time (bring-up step 5)."}},
        {4,
         {4, "IR BUDGET", "The infrared emitters were on longer than their allowed time budget.",
          "Check the IR scheduler settings and the emitter timer; confirm the emitters turn off "
          "(step 7)."}},
        {5,
         {5, "IR STUCK ON", "An IR emitter appears to stay lit.",
          "Power off at once. Check the emitter driver transistors and the emitter-off timer "
          "wiring."}},
        {6,
         {6, "BATTERY LOW",
          "The pack voltage fell below the cutoff (there is no hardware cutoff on RC3.1).",
          "Stop, charge or replace the pack. Check the divider calibration (step 8) if the voltage "
          "looks wrong."}},
        {7,
         {7, "SELFTEST FAILED", "The power-on self-test failed.",
          "Read which test failed on the OLED / debugger; check the IMU, flash and the ADC "
          "readings."}},
        {8,
         {8, "WHEEL STALL", "A wheel was driven hard but did not turn.",
          "Check for a blocked wheel or gear, the encoder wiring and motor wires; keep the wheels "
          "lifted for tests."}},
        {9,
         {9, "SENSOR INVALID", "A sensor needed for the run (IMU or IR) gave invalid data.",
          "Check the IMU wiring and 3V3; look at the status line (imu=, irv=)."}},
        {10,
         {10, "CALIBRATION TIMEOUT",
          "The gyro bias calibration did not finish (the robot was moved).",
          "Keep the robot still on a flat surface while it calibrates; retry."}},
        {11,
         {11, "MOTOR DRIVER",
          "The DRV8833 nFAULT line was asserted (over-current, over-temperature or under-voltage).",
          "Power off, check motor wires and short circuits, let the driver cool, then clear the "
          "fault."}},
    };
    if (const auto it = table.find(code); it != table.end()) return it->second;
    return {code, "UNKNOWN (" + std::to_string(code) + ")",
            "Not a fault code this firmware defines.",
            "Check that the dashboard and the firmware are the same version."};
}

FaultInfo sim_fault_info(const std::string& kind, const std::string& detail) {
    struct Card {
        const char* key;
        const char* name;
        const char* meaning;
        const char* action;
    };
    static const Card cards[] = {
        {"faulted", "Firmware fault",
         "The firmware used the HAL in a way real hardware cannot (for example a blocking call "
         "inside the 1 kHz tick).",
         "Read the fault text, fix the call site, rebuild; the simulator reloads the DLL "
         "(Ctrl+R)."},
        {"hang", "Firmware not responding",
         "fw_main stopped yielding to the simulator (a busy wait without hal_wait_* / hal_yield).",
         "Replace spin loops with hal_wait_us / hal_yield, or raise the watchdog limit for slow "
         "debug builds."},
        {"crash", "Wall contact",
         "The robot body touched a wall or post. Real robots usually lose position here.",
         "Check wall-centring gains, turn entry speed and sensor calibration; scrub the timeline "
         "to the contact."},
        {"load error", "Firmware did not load",
         "The DLL or maze could not be opened, or it does not export fw_main / the HAL table.",
         "Rebuild the firmware target, check the path, and look at the log for the exact loader "
         "message."},
        {"time limit", "Run timed out", "The mission did not finish within the time limit.",
         "Look at the event log: a search that loops, or a mouse waiting for a button press."},
        {"halted", "Firmware stopped", "fw_main returned before the mission finished.",
         "Firmware main loops normally never return; check the error path that exits."},
        {"error", "Firmware reported an error",
         "The firmware logged an ERROR line (for example: target unreachable).",
         "Read the log line; a wrong wall map after a crash is the usual cause."},
        {"brownout", "Battery brownout",
         "The predicted pack voltage fell below the MCU reset level under motor load.",
         "Reduce peak current (gentler acceleration), use a fresher pack, or lower R_int in the "
         "robot config."},
        {"overvoltage", "Motor voltage capped",
         "The firmware asked for more than the motor's rated voltage; the simulator clamped it.",
         "Scale duty by V_motor_max / V_bat (6.0 V cap for N20 on 2S) and never drive 100 % PWM."},
    };
    for (const Card& c : cards)
        if (kind == c.key)
            return {0, c.name,
                    detail.empty() ? c.meaning : std::string(c.meaning) + " (" + detail + ")",
                    c.action};
    return {0, "Problem: " + kind,
            detail.empty() ? "No explanation is recorded for this outcome." : detail,
            "Check the log panel for details."};
}

std::vector<TelemetryEvent> telemetry_events(const std::vector<LogRow>& rows, std::size_t limit) {
    std::vector<TelemetryEvent> out;
    const LogRow* prev = nullptr;
    std::optional<int> seg;
    for (const LogRow& r : rows) {
        if (!seg || r.segment != *seg) {
            seg = r.segment;
            prev = nullptr;
            out.push_back({0.0, r.segment, "segment",
                           "log segment " + std::to_string(r.segment) + " starts (state " +
                               r.state_name + ")"});
        }
        const double t = r.t_ms / 1000.0;
        if (prev) {
            if (r.state != prev->state)
                out.push_back({t, r.segment, "state", prev->state_name + " -> " + r.state_name});
            if (r.fault != prev->fault)
                out.push_back(
                    {t, r.segment, "fault", "fault " + prev->fault_name + " -> " + r.fault_name});
            const std::pair<bool LogRow::*, const char*> flags[] = {
                {&LogRow::bias_valid, "gyro bias"},
                {&LogRow::slip, "wheel slip"},
                {&LogRow::degraded, "gyro-less mode"}};
            const char* keys[] = {"bias_valid", "slip", "degraded"};
            for (int k = 0; k < 3; ++k)
                if (r.*flags[k].first != prev->*flags[k].first)
                    out.push_back(
                        {t, r.segment, keys[k],
                         std::string(flags[k].second) + (r.*flags[k].first ? " on" : " off")});
        }
        prev = &r;
        if (out.size() >= limit) break;
    }
    return out;
}

}  // namespace mmsim::core::analysis
