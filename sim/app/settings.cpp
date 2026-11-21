#include "sim/app/settings.hpp"

#include <fstream>
#include <iterator>
#include <nlohmann/json.hpp>

namespace mmsim::app {

namespace fs = std::filesystem;
using nlohmann::json;

namespace {

template <typename T>
void get(const json& j, const char* key, T& dst) {
    const auto it = j.find(key);
    if (it == j.end()) return;
    try {
        dst = it->get<T>();
    } catch (const json::exception&) {
        // wrong type: keep the default
    }
}

void get_path(const json& j, const char* key, fs::path& dst) {
    std::string s;
    get(j, key, s);
    if (!s.empty()) dst = fs::path(s);
}

}  // namespace

bool load_settings(const fs::path& file, AppSettings& out, std::string& error) {
    out = AppSettings{};
    error.clear();
    std::error_code ec;
    if (!fs::exists(file, ec)) return true;
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        error = "cannot read " + file.string();
        return false;
    }
    const std::string text((std::istreambuf_iterator<char>(in)), {});
    json j;
    try {
        j = json::parse(text);
    } catch (const json::parse_error& e) {
        error = file.string() + ": " + e.what();
        return false;
    }
    if (!j.is_object()) {
        error = file.string() + ": top level must be an object";
        return false;
    }
    get_path(j, "maze", out.maze);
    get_path(j, "config", out.config);
    get_path(j, "firmware", out.firmware);
    get_path(j, "mms_solver", out.mms_solver);
    get(j, "mms_sensor_walls", out.mms_sensor_walls);
    get(j, "seed", out.seed);
    get(j, "speed", out.speed);
    if (out.speed < 0) out.speed = 1.0;
    get(j, "boot_button0", out.boot_buttons[0]);
    get(j, "boot_button1", out.boot_buttons[1]);
    get(j, "workspace", out.workspace);
    get(j, "mode", out.mode);
    get(j, "muted", out.muted);
    get(j, "layout_version", out.layout_version);
    get(j, "timer_audio", out.timer_audio);
    get(j, "details_open", out.details_open);
    get(j, "timer_minutes", out.timer_minutes);
    return true;
}

bool save_settings(const fs::path& file, const AppSettings& s, std::string& error) {
    json j;
    j["version"] = 1;
    j["maze"] = s.maze.generic_string();
    j["config"] = s.config.generic_string();
    j["firmware"] = s.firmware.generic_string();
    j["mms_solver"] = s.mms_solver.generic_string();
    j["mms_sensor_walls"] = s.mms_sensor_walls;
    j["seed"] = s.seed;
    j["speed"] = s.speed;
    j["boot_button0"] = s.boot_buttons[0];
    j["boot_button1"] = s.boot_buttons[1];
    j["workspace"] = s.workspace;
    j["mode"] = s.mode;
    j["muted"] = s.muted;
    j["layout_version"] = s.layout_version;
    j["timer_audio"] = s.timer_audio;
    j["details_open"] = s.details_open;
    j["timer_minutes"] = s.timer_minutes;
    const fs::path tmp = fs::path(file).concat(".tmp");
    {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        if (!f) {
            error = "cannot write " + tmp.string();
            return false;
        }
        f << j.dump(2) << "\n";
        if (!f) {
            error = "write failed: " + tmp.string();
            return false;
        }
    }
    std::error_code ec;
    fs::rename(tmp, file, ec);
    if (ec) {
        error = "cannot replace " + file.string() + ": " + ec.message();
        fs::remove(tmp, ec);
        return false;
    }
    return true;
}

}  // namespace mmsim::app
