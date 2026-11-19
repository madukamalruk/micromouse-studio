#include "sim/core/robot_config.hpp"

#include <fstream>
#include <functional>
#include <nlohmann/json.hpp>
#include <sstream>
#include <vector>

namespace mmsim::core {

namespace {

using nlohmann::json;

enum class Rule { Positive, NonNegative, Fraction01, UnitInterval };

struct Field {
    const char* key;  // dotted path, e.g. "motor.gear_ratio"
    std::function<double&(RobotConfig&)> ref;
    Rule rule;
};

const std::vector<Field>& fields() {
    static const std::vector<Field> f = {
        {"mass_kg", [](RobotConfig& c) -> double& { return c.mass; }, Rule::Positive},
        {"yaw_inertia_kg_m2", [](RobotConfig& c) -> double& { return c.yaw_inertia; },
         Rule::Positive},
        {"track_width_m", [](RobotConfig& c) -> double& { return c.track_width; }, Rule::Positive},
        {"wheel.radius_m", [](RobotConfig& c) -> double& { return c.wheel_radius; },
         Rule::Positive},
        {"wheel.inertia_kg_m2", [](RobotConfig& c) -> double& { return c.wheel_inertia; },
         Rule::NonNegative},
        {"body.length_m", [](RobotConfig& c) -> double& { return c.body.length; }, Rule::Positive},
        {"body.width_m", [](RobotConfig& c) -> double& { return c.body.width; }, Rule::Positive},
        {"body.offset_x_m", [](RobotConfig& c) -> double& { return c.body.offset_x; },
         Rule::NonNegative},
        {"motor.resistance_ohm", [](RobotConfig& c) -> double& { return c.motor.resistance; },
         Rule::Positive},
        {"motor.kt_nm_per_a", [](RobotConfig& c) -> double& { return c.motor.kt; }, Rule::Positive},
        {"motor.ke_v_s_per_rad", [](RobotConfig& c) -> double& { return c.motor.ke; },
         Rule::Positive},
        {"motor.gear_ratio", [](RobotConfig& c) -> double& { return c.motor.gear_ratio; },
         Rule::Positive},
        {"motor.rotor_inertia_kg_m2",
         [](RobotConfig& c) -> double& { return c.motor.rotor_inertia; }, Rule::NonNegative},
        {"motor.viscous_nm_s_per_rad", [](RobotConfig& c) -> double& { return c.motor.viscous; },
         Rule::NonNegative},
        {"motor.coulomb_nm", [](RobotConfig& c) -> double& { return c.motor.coulomb; },
         Rule::NonNegative},
        {"motor.efficiency", [](RobotConfig& c) -> double& { return c.motor.efficiency; },
         Rule::Fraction01},
        {"motor.max_voltage_v", [](RobotConfig& c) -> double& { return c.motor.max_voltage; },
         Rule::Positive},
        {"battery_voltage_v", [](RobotConfig& c) -> double& { return c.battery_voltage; },
         Rule::Positive},
        {"encoder_counts_per_motor_rev",
         [](RobotConfig& c) -> double& { return c.encoder_counts_per_motor_rev; }, Rule::Positive},
        {"collision_restitution", [](RobotConfig& c) -> double& { return c.collision_restitution; },
         Rule::UnitInterval},
    };
    return f;
}

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

RobotConfigResult fail(std::string msg) {
    return {std::nullopt, "robot config: " + std::move(msg)};
}

}  // namespace

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

    for (const Field& f : fields()) {
        const json* v = lookup(j, f.key);
        if (!v) return fail(std::string("missing key '") + f.key + "'");
        if (!v->is_number()) return fail(std::string("key '") + f.key + "' must be a number");
        const double x = v->get<double>();
        if (!rule_ok(f.rule, x))
            return fail(std::string("key '") + f.key + "' " + rule_text(f.rule) + " (got " +
                        fmt(x) + ")");
        f.ref(c) = x;
    }
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

std::string robot_config_to_json(const RobotConfig& cfg) {
    RobotConfig c = cfg;  // Field::ref needs a mutable object
    json j;
    j["name"] = c.name;
    if (!c.description.empty()) j["description"] = c.description;
    for (const Field& f : fields()) {
        const std::string key = f.key;
        const auto dot = key.find('.');
        if (dot == std::string::npos) {
            j[key] = f.ref(c);
        } else {
            j[key.substr(0, dot)][key.substr(dot + 1)] = f.ref(c);
        }
    }
    return j.dump(2);
}

}  // namespace mmsim::core
