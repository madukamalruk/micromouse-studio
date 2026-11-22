#include "sim/app/config_edit.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>

namespace mmsim::app {

namespace fs = std::filesystem;

ConfigEdit edit_from_config(const core::RobotConfig& c) {
    ConfigEdit e;
    e.mass_g = c.mass * 1000.0;
    e.wheel_radius_mm = c.wheel_radius * 1000.0;
    e.track_width_mm = c.track_width * 1000.0;
    e.max_voltage = c.motor.max_voltage;
    e.slip_enabled = c.slip.enabled;
    e.tyre_mu = c.slip.mu;
    e.battery_model = c.battery.model;
    e.r_int_ohm = c.battery.r_int_ohm;
    e.has_sensors = !c.sensors.empty();
    if (e.has_sensors) {
        const auto& s = c.sensors.front();
        e.sensor_range_mm = s.range_max * 1000.0;
        e.noise_sigma = s.noise_sigma;
        e.noise_rel_pct = s.noise_rel * 100.0;
    }
    e.orig_range_mm = e.sensor_range_mm;
    e.orig_sigma = e.noise_sigma;
    e.orig_rel_pct = e.noise_rel_pct;
    return e;
}

namespace {
/// CON, PRN, AUX, NUL, COM1-9, LPT1-9 (any case, with or without an extension).
bool is_reserved_windows_name(const std::string& stem) {
    std::string s = stem.substr(0, stem.find('.'));
    while (!s.empty() && s.back() == ' ') s.pop_back();
    for (char& ch : s) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    if (s == "CON" || s == "PRN" || s == "AUX" || s == "NUL") return true;
    return s.size() == 4 && (s.compare(0, 3, "COM") == 0 || s.compare(0, 3, "LPT") == 0) &&
           s[3] >= '1' && s[3] <= '9';
}

bool changed(double a, double b) {
    return std::fabs(a - b) > 1e-9;
}
}  // namespace

void apply_edit(const ConfigEdit& e, core::RobotConfig& c) {
    // Compare in the editor's units against what edit_from_config would give, so an untouched
    // field keeps the exact original double (no mm/m round trip).
    const ConfigEdit base = edit_from_config(c);
    if (changed(e.mass_g, base.mass_g)) c.mass = e.mass_g / 1000.0;
    if (changed(e.wheel_radius_mm, base.wheel_radius_mm))
        c.wheel_radius = e.wheel_radius_mm / 1000.0;
    if (changed(e.track_width_mm, base.track_width_mm)) c.track_width = e.track_width_mm / 1000.0;
    c.motor.max_voltage = c.motor.allow_unsafe_voltage
                              ? e.max_voltage
                              : std::min(e.max_voltage, core::kMotorSafeVoltage);
    c.slip.enabled = e.slip_enabled;
    if (changed(e.tyre_mu, base.tyre_mu)) c.slip.mu = e.tyre_mu;
    c.battery.model = e.battery_model;
    if (changed(e.r_int_ohm, base.r_int_ohm)) c.battery.r_int_ohm = e.r_int_ohm;
    if (e.has_sensors) {
        const bool range = changed(e.sensor_range_mm, e.orig_range_mm);
        const bool sigma = changed(e.noise_sigma, e.orig_sigma);
        const bool rel = changed(e.noise_rel_pct, e.orig_rel_pct);
        for (auto& s : c.sensors) {
            if (range) s.range_max = e.sensor_range_mm / 1000.0;
            if (sigma) s.noise_sigma = e.noise_sigma;
            if (rel) s.noise_rel = e.noise_rel_pct / 100.0;
        }
    }
}

std::vector<ConfigIssue> validate_edit(const ConfigEdit& e) {
    std::vector<ConfigIssue> v;
    auto err = [&](std::string t) { v.push_back({IssueLevel::Error, std::move(t)}); };
    if (!(e.mass_g > 0.0)) err("Mass must be above 0 g.");
    if (!(e.wheel_radius_mm > 0.0)) err("Wheel radius must be above 0 mm.");
    if (!(e.track_width_mm > 0.0)) err("Track width must be above 0 mm.");
    if (!(e.max_voltage > 0.0)) err("Motor max voltage must be above 0 V.");
    if (e.slip_enabled && !(e.tyre_mu > 0.0)) err("Tyre mu must be above 0.");
    if (e.r_int_ohm < 0.0) err("Battery internal resistance cannot be negative.");
    if (e.has_sensors && !(e.sensor_range_mm > 0.0)) err("Sensor range must be above 0 mm.");
    if (e.noise_sigma < 0.0 || e.noise_rel_pct < 0.0) err("Sensor noise cannot be negative.");
    if (e.max_voltage > kMotorVoltageLimit)
        err("Motor voltage above 6.0 V is not allowed: N20 motors on a 2S pack must stay at or "
            "below 6.0 V (the simulation caps it).");
    return v;
}

bool has_errors(const std::vector<ConfigIssue>& issues) {
    for (const auto& i : issues)
        if (i.level == IssueLevel::Error) return true;
    return false;
}

std::string unique_copy_stem(const fs::path& dir, const std::string& base) {
    std::string b = base.empty() ? "robot" : base;
    // Strip a previous "_edited[N]" so copies of copies do not grow a tail.
    const auto pos = b.rfind("_edited");
    if (pos != std::string::npos) {
        bool digits = true;
        for (std::size_t i = pos + 7; i < b.size(); ++i)
            if (b[i] < '0' || b[i] > '9') digits = false;
        if (digits) b.resize(pos);
    }
    for (int n = 1;; ++n) {
        const std::string stem = b + "_edited" + (n == 1 ? "" : std::to_string(n));
        std::error_code ec;
        if (!fs::exists(dir / (stem + ".json"), ec)) return stem;
    }
}

bool save_config_copy(const core::RobotConfig& c, const fs::path& dir, const std::string& stem,
                      fs::path& out, std::string& error) {
    error.clear();
    if (stem.empty() || stem.find_first_of("\\/:*?\"<>|") != std::string::npos) {
        error = "The file name has characters Windows does not allow.";
        return false;
    }
    if (is_reserved_windows_name(stem)) {
        error = "'" + stem + "' is a reserved Windows device name; pick another name.";
        return false;
    }

    std::error_code ec;
    fs::create_directories(dir, ec);
    out = dir / (stem + ".json");
    if (fs::exists(out, ec)) {
        error =
            out.filename().string() + " already exists; pick another name (nothing was changed).";
        return false;
    }
    core::RobotConfig copy = c;
    copy.name = stem;
    std::ofstream f(out, std::ios::binary | std::ios::trunc);
    if (!f) {
        error = "Cannot write " + out.string();
        return false;
    }
    f << core::robot_config_to_json(copy) << "\n";
    if (!f) {
        error = "Write failed: " + out.string();
        return false;
    }
    return true;
}

}  // namespace mmsim::app
