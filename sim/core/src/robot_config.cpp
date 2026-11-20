#include "sim/core/robot_config.hpp"

#include <cmath>
#include <fstream>
#include <functional>
#include <nlohmann/json.hpp>
#include <sstream>
#include <vector>

namespace mmsim::core {

namespace {

using nlohmann::json;

enum class Rule { Positive, NonNegative, Fraction01, UnitInterval, Any };

template <class T>
struct Field {
    const char* key;  // dotted path, e.g. "motor.gear_ratio"
    std::function<double&(T&)> ref;
    Rule rule;
    bool required = true;
};

template <class T>
struct Flag {
    const char* key;
    std::function<bool&(T&)> ref;
};

#define F(path, expr, rule) {path, [](RobotConfig& c) -> double& { return expr; }, rule, true}
#define O(path, expr, rule) {path, [](RobotConfig& c) -> double& { return expr; }, rule, false}

const std::vector<Field<RobotConfig>>& fields() {
    static const std::vector<Field<RobotConfig>> f = {
        F("mass_kg", c.mass, Rule::Positive),
        F("yaw_inertia_kg_m2", c.yaw_inertia, Rule::Positive),
        F("track_width_m", c.track_width, Rule::Positive),
        F("wheel.radius_m", c.wheel_radius, Rule::Positive),
        F("wheel.inertia_kg_m2", c.wheel_inertia, Rule::NonNegative),
        F("body.length_m", c.body.length, Rule::Positive),
        F("body.width_m", c.body.width, Rule::Positive),
        F("body.offset_x_m", c.body.offset_x, Rule::NonNegative),
        F("motor.resistance_ohm", c.motor.resistance, Rule::Positive),
        F("motor.kt_nm_per_a", c.motor.kt, Rule::Positive),
        F("motor.ke_v_s_per_rad", c.motor.ke, Rule::Positive),
        F("motor.gear_ratio", c.motor.gear_ratio, Rule::Positive),
        F("motor.rotor_inertia_kg_m2", c.motor.rotor_inertia, Rule::NonNegative),
        F("motor.viscous_nm_s_per_rad", c.motor.viscous, Rule::NonNegative),
        F("motor.coulomb_nm", c.motor.coulomb, Rule::NonNegative),
        F("motor.efficiency", c.motor.efficiency, Rule::Fraction01),
        F("motor.max_voltage_v", c.motor.max_voltage, Rule::Positive),
        F("battery_voltage_v", c.battery_voltage, Rule::Positive),
        F("encoder.counts_per_motor_rev", c.encoder.counts_per_motor_rev, Rule::Positive),
        F("collision_restitution", c.collision_restitution, Rule::UnitInterval),
        O("imu.gyro_noise_dps", c.imu.gyro_noise_dps, Rule::NonNegative),
        O("imu.gyro_bias_dps", c.imu.gyro_bias_dps, Rule::Any),
        O("imu.gyro_bias_walk_dps_per_sqrt_s", c.imu.gyro_bias_walk, Rule::NonNegative),
        O("imu.gyro_quant_dps", c.imu.gyro_quant_dps, Rule::NonNegative),
        O("imu.gyro_range_dps", c.imu.gyro_range_dps, Rule::Positive),
        O("imu.accel_noise_mps2", c.imu.accel_noise_mps2, Rule::NonNegative),
        O("imu.accel_quant_mps2", c.imu.accel_quant_mps2, Rule::NonNegative),
        O("imu.accel_range_mps2", c.imu.accel_range_mps2, Rule::Positive),
        O("battery.cells", c.battery.cells, Rule::Positive),
        O("battery.capacity_mah", c.battery.capacity_mah, Rule::Positive),
        O("battery.r_int_ohm", c.battery.r_int_ohm, Rule::NonNegative),
        O("battery.soc0", c.battery.soc0, Rule::Fraction01),
        O("battery.electronics_a", c.battery.electronics_a, Rule::NonNegative),
        O("battery.reset_v", c.battery.reset_v, Rule::Positive),
        O("slip.mu", c.slip.mu, Rule::Positive),
        O("slip.long_stiffness_n_per_mps", c.slip.long_stiffness, Rule::Positive),
        O("slip.slip_gradient_deg_per_g", c.slip.slip_gradient_deg_per_g, Rule::NonNegative),
    };
    return f;
}

const std::vector<Flag<RobotConfig>>& flags() {
    static const std::vector<Flag<RobotConfig>> f = {
        {"encoder.invert_left", [](RobotConfig& c) -> bool& { return c.encoder.invert_left; }},
        {"encoder.invert_right", [](RobotConfig& c) -> bool& { return c.encoder.invert_right; }},
        {"slip.enabled", [](RobotConfig& c) -> bool& { return c.slip.enabled; }},
        {"battery.model", [](RobotConfig& c) -> bool& { return c.battery.model; }},
    };
    return f;
}

#undef F
#undef O
#define S(path, expr, rule) {path, [](SensorConfig& s) -> double& { return expr; }, rule, false}

const std::vector<Field<SensorConfig>>& sensor_fields() {
    static const std::vector<Field<SensorConfig>> f = {
        S("x_m", s.x, Rule::Any),
        S("y_m", s.y, Rule::Any),
        S("angle_deg", s.angle_deg, Rule::Any),
        S("fov_deg", s.fov_deg, Rule::NonNegative),
        S("edge_weight", s.edge_weight, Rule::Fraction01),
        S("range_min_m", s.range_min, Rule::NonNegative),
        S("range_max_m", s.range_max, Rule::Positive),
        S("rate_hz", s.rate_hz, Rule::Positive),
        S("latency_ms", s.latency_ms, Rule::NonNegative),
        S("noise_sigma", s.noise_sigma, Rule::NonNegative),
        S("noise_rel", s.noise_rel, Rule::NonNegative),
        S("quant", s.quant, Rule::NonNegative),
        S("ir_gain", s.ir_gain, Rule::Positive),
        S("ir_d0_m", s.ir_d0, Rule::NonNegative),
        S("ir_ambient", s.ir_ambient, Rule::NonNegative),
        S("ir_adc_max", s.ir_adc_max, Rule::Positive),
        S("ir_incidence_exp", s.ir_incidence_exp, Rule::NonNegative),
    };
    return f;
}
#undef S

std::string fmt(double v) {
    std::ostringstream o;
    o << v;
    return o.str();
}

const char* rule_text(Rule r) {
    switch (r) {
        case Rule::Positive:
            return "must be > 0";
        case Rule::NonNegative:
            return "must be >= 0";
        case Rule::Fraction01:
            return "must be in (0, 1]";
        case Rule::UnitInterval:
            return "must be in [0, 1]";
        case Rule::Any:
            return "";
    }
    return "";
}

bool rule_ok(Rule r, double v) {
    switch (r) {
        case Rule::Positive:
            return v > 0.0;
        case Rule::NonNegative:
            return v >= 0.0;
        case Rule::Fraction01:
            return v > 0.0 && v <= 1.0;
        case Rule::UnitInterval:
            return v >= 0.0 && v <= 1.0;
        case Rule::Any:
            return true;
    }
    return false;
}

/// Walks "a.b" into j; returns nullptr if any part is missing.
const json* lookup(const json& j, const std::string& dotted) {
    const json* cur = &j;
    std::size_t start = 0;
    while (true) {
        const auto dot = dotted.find('.', start);
        const std::string part =
            dotted.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
        if (!cur->is_object()) return nullptr;
        const auto it = cur->find(part);
        if (it == cur->end()) return nullptr;
        cur = &*it;
        if (dot == std::string::npos) return cur;
        start = dot + 1;
    }
}

/// Reads all numeric fields of T from j. Returns an error message naming prefix+key, or "".
template <class T>
std::string read_fields(const json& j, const std::vector<Field<T>>& fs, T& out,
                        const std::string& prefix) {
    for (const auto& f : fs) {
        const json* v = lookup(j, f.key);
        const std::string key = prefix + f.key;
        if (!v) {
            if (f.required) return "missing key '" + key + "'";
            continue;
        }
        if (!v->is_number()) return "key '" + key + "' must be a number";
        const double x = v->template get<double>();
        if (!std::isfinite(x)) return "key '" + key + "' must be finite";
        if (!rule_ok(f.rule, x))
            return "key '" + key + "' " + rule_text(f.rule) + " (got " + fmt(x) + ")";
        f.ref(out) = x;
    }
    return {};
}

std::optional<SensorType> parse_type(const json& v) {
    if (!v.is_string()) return std::nullopt;
    const auto s = v.get<std::string>();
    if (s == "ir") return SensorType::IR;
    if (s == "tof") return SensorType::ToF;
    return std::nullopt;
}

RobotConfigResult fail(std::string msg) {
    return {std::nullopt, "robot config: " + std::move(msg)};
}

}  // namespace

SensorConfig default_sensor(SensorType type) {
    SensorConfig s;
    s.type = type;
    if (type == SensorType::IR) {
        s.fov_deg = 20.0;
        s.rays = 5;
        s.range_min = 0.0;
        s.range_max = 0.25;
        s.rate_hz = 1000.0;
        s.latency_ms = 0.0;
        s.noise_sigma = 4.0;  // ADC counts
        s.noise_rel = 0.01;
        s.quant = 1.0;
    } else {  // VL53L0X-class time of flight
        s.fov_deg = 25.0;
        s.rays = 5;
        s.range_min = 0.02;
        s.range_max = 0.5;
        s.rate_hz = 50.0;
        s.latency_ms = 20.0;
        s.noise_sigma = 2.0;  // mm
        s.noise_rel = 0.01;
        s.quant = 1.0;
    }
    return s;
}

std::optional<std::vector<SensorConfig>> sensor_layout_preset(std::string_view preset,
                                                              SensorType type,
                                                              const BodyConfig& body) {
    const double front = body.offset_x + body.length / 2.0;  // nose, from the axle
    struct Mount {
        const char* name;
        double x, y, angle;
    };
    std::vector<Mount> m;
    if (preset == "front_side") {
        m = {{"left", front - 0.028, 0.030, 90.0},
             {"front_left", front - 0.0075, 0.025, 0.0},
             {"front_right", front - 0.0075, -0.025, 0.0},
             {"right", front - 0.028, -0.030, -90.0}};
    } else if (preset == "front_diagonal") {
        m = {{"diag_left", front - 0.012, 0.020, 45.0},
             {"front_left", front - 0.0075, 0.025, 0.0},
             {"front_right", front - 0.0075, -0.025, 0.0},
             {"diag_right", front - 0.012, -0.020, -45.0}};
    } else {
        return std::nullopt;
    }
    std::vector<SensorConfig> out;
    for (const auto& mt : m) {
        SensorConfig s = default_sensor(type);
        s.name = mt.name;
        s.x = mt.x;
        s.y = mt.y;
        s.angle_deg = mt.angle;
        out.push_back(s);
    }
    return out;
}

RobotConfigResult parse_robot_config(std::string_view text) {
    json j;
    try {
        j = json::parse(text);
    } catch (const json::parse_error& e) {
        return fail(std::string("invalid JSON: ") + e.what());
    }
    if (!j.is_object()) return fail("top level must be a JSON object");

    RobotConfig c;
    const json* name = lookup(j, "name");
    if (!name) return fail("missing key 'name'");
    if (!name->is_string()) return fail("key 'name' must be a string");
    c.name = name->get<std::string>();
    if (const json* d = lookup(j, "description"); d && d->is_string())
        c.description = d->get<std::string>();

    if (auto e = read_fields(j, fields(), c, ""); !e.empty()) return fail(e);
    for (const auto& f : flags()) {
        const json* v = lookup(j, f.key);
        if (!v) continue;
        if (!v->is_boolean()) return fail(std::string("key '") + f.key + "' must be true/false");
        f.ref(c) = v->get<bool>();
    }

    // Sensors: a preset layout and/or an explicit list (the list wins if both are given).
    if (const json* lay = lookup(j, "sensor_layout")) {
        const json* p = lookup(*lay, "preset");
        if (!p || !p->is_string()) return fail("key 'sensor_layout.preset' must be a string");
        SensorType type = SensorType::IR;
        if (const json* t = lookup(*lay, "type")) {
            auto pt = parse_type(*t);
            if (!pt) return fail("key 'sensor_layout.type' must be \"ir\" or \"tof\"");
            type = *pt;
        }
        auto preset = sensor_layout_preset(p->get<std::string>(), type, c.body);
        if (!preset)
            return fail("key 'sensor_layout.preset' must be \"front_side\" or \"front_diagonal\"");
        c.sensors = *preset;
        // Optional overrides applied to every preset sensor (e.g. noise, fov, range).
        if (const json* ov = lookup(*lay, "overrides")) {
            for (auto& s : c.sensors)
                if (auto e = read_fields(*ov, sensor_fields(), s, "sensor_layout.overrides.");
                    !e.empty())
                    return fail(e);
            if (const json* r = lookup(*ov, "rays")) {
                if (!r->is_number_integer() || r->get<int>() < 1 || r->get<int>() > 64)
                    return fail("key 'sensor_layout.overrides.rays' must be an integer 1..64");
                for (auto& s : c.sensors) s.rays = r->get<int>();
            }
        }
    }
    if (const json* list = lookup(j, "sensors")) {
        if (!list->is_array()) return fail("key 'sensors' must be an array");
        c.sensors.clear();
        for (std::size_t i = 0; i < list->size(); ++i) {
            const json& sj = (*list)[i];
            const std::string pre = "sensors[" + std::to_string(i) + "].";
            if (!sj.is_object())
                return fail("key '" + pre.substr(0, pre.size() - 1) + "' must be an object");
            const json* t = lookup(sj, "type");
            auto type = t ? parse_type(*t) : std::nullopt;
            if (!type) return fail("key '" + pre + "type' must be \"ir\" or \"tof\"");
            SensorConfig s = default_sensor(*type);
            if (const json* n = lookup(sj, "name"); n && n->is_string())
                s.name = n->get<std::string>();
            if (auto e = read_fields(sj, sensor_fields(), s, pre); !e.empty()) return fail(e);
            if (const json* r = lookup(sj, "rays")) {
                if (!r->is_number_integer() || r->get<int>() < 1 || r->get<int>() > 64)
                    return fail("key '" + pre + "rays' must be an integer 1..64");
                s.rays = r->get<int>();
            }
            c.sensors.push_back(s);
        }
    }
    if (c.sensors.size() > 16) return fail("key 'sensors': at most 16 sensors");
    for (std::size_t i = 0; i < c.sensors.size(); ++i)
        if (c.sensors[i].range_min >= c.sensors[i].range_max)
            return fail("key 'sensors[" + std::to_string(i) +
                        "].range_min_m' must be < range_max_m");
    return {c, {}};
}

RobotConfigResult load_robot_config(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return fail("cannot open " + path.string());
    std::ostringstream ss;
    ss << in.rdbuf();
    auto r = parse_robot_config(ss.str());
    if (!r.ok()) r.error += " [" + path.filename().string() + "]";
    return r;
}

namespace {
void put(json& j, const std::string& key, const json& v) {
    const auto dot = key.find('.');
    if (dot == std::string::npos)
        j[key] = v;
    else
        j[key.substr(0, dot)][key.substr(dot + 1)] = v;
}
}  // namespace

std::string robot_config_to_json(const RobotConfig& cfg) {
    RobotConfig c = cfg;  // Field::ref needs a mutable object
    json j;
    j["name"] = c.name;
    if (!c.description.empty()) j["description"] = c.description;
    for (const auto& f : fields()) put(j, f.key, f.ref(c));
    for (const auto& f : flags()) put(j, f.key, f.ref(c));
    json list = json::array();
    for (SensorConfig s : c.sensors) {
        json sj;
        sj["name"] = s.name;
        sj["type"] = s.type == SensorType::IR ? "ir" : "tof";
        sj["rays"] = s.rays;
        for (const auto& f : sensor_fields()) sj[f.key] = f.ref(s);
        list.push_back(sj);
    }
    j["sensors"] = list;
    return j.dump(2);
}

}  // namespace mmsim::core
