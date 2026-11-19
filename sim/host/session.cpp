#include "sim/host/session.hpp"

#include <algorithm>
#include <utility>

namespace mmsim::host {

Session::Session(core::Maze maze, core::RobotConfig cfg, core::MazeDims dims, std::uint64_t seed)
    : world_(std::move(maze), std::move(cfg), dims, seed), host_(*this) {}

Session::~Session() = default;

bool Session::load(const std::filesystem::path& dll) {
    world_.reset();
    clear_outputs();
    return host_.load(dll);
}

void Session::reset() {
    world_.reset();
    clear_outputs();
    host_.reload();
}

void Session::unload() {
    host_.unload();
    world_.reset();
    clear_outputs();
}

void Session::clear_outputs() {
    logs_.clear();
    leds_.clear();
    telemetry_.clear();
    ann_ = {};
    for (bool& l : led_state_) l = false;
}

void Session::set_button(int btn, bool pressed) {
    if (btn >= 0 && btn < 8) buttons_[btn] = pressed;
}

void Session::step_tick() {
    if (host_.state() == FwState::Ready) host_.start(world_.time_us());
    world_.step_tick();
    host_.tick(world_.time_us());
}

void Session::run_ms(std::uint32_t ms) {
    for (std::uint32_t i = 0; i < ms; ++i) step_tick();
}

std::optional<float> Session::telemetry_value(const std::string& channel) const {
    const auto it = telemetry_.find(channel);
    if (it == telemetry_.end() || it->second.empty()) return std::nullopt;
    return it->second.back().value;
}

// --- HalBackend ---------------------------------------------------------------

void Session::motor_set_duty(int side, float duty) {
    world_.set_motor_duty(side, duty);
}
float Session::battery_voltage() {
    return static_cast<float>(world_.config().battery_voltage);
}
std::int32_t Session::encoder_count(int side) {
    return world_.encoder_count(side);
}
// Firmware sees the measured IMU (noise, bias, quantisation), never ground truth.
float Session::gyro_z_dps() {
    return static_cast<float>(world_.imu().gyro_dps[2]);
}
void Session::imu(float gyro_dps[3], float accel_mps2[3]) {
    const core::ImuReading& r = world_.imu();
    for (int i = 0; i < 3; ++i) {
        gyro_dps[i] = static_cast<float>(r.gyro_dps[i]);
        accel_mps2[i] = static_cast<float>(r.accel_mps2[i]);
    }
}
std::uint8_t Session::sensor_count() {
    return static_cast<std::uint8_t>(std::min<std::size_t>(world_.sensor_count(), 255));
}
std::uint16_t Session::sensor_read(std::uint8_t ch) {
    return world_.sensor_raw(ch);
}
std::uint16_t Session::sensor_distance(std::uint8_t ch) {
    return world_.sensor_distance_mm(ch);
}
void Session::emitter_set(std::uint8_t ch, bool on) {
    world_.set_emitter(ch, on);
}

void Session::led_set(std::uint8_t led, bool on) {
    if (led < 8) led_state_[led] = on;
    leds_.push_back({world_.time_us(), led, on});
}

bool Session::button_pressed(std::uint8_t btn) {
    return btn < 8 && buttons_[btn];
}

void Session::log(const char* line) {
    logs_.push_back({world_.time_us(), line});
    if (logs_.size() > kMaxLogLines) logs_.pop_front();
}

void Session::telemetry(const char* channel, float value) {
    auto& q = telemetry_[channel];
    q.push_back({world_.time_us(), value});
    if (q.size() > kTelemetryHistory) q.pop_front();
}

void Session::annotate_cell_color(std::uint8_t x, std::uint8_t y, char color) {
    ann_.color[{x, y}] = color;
}
void Session::annotate_cell_text(std::uint8_t x, std::uint8_t y, const char* text) {
    ann_.text[{x, y}] = text;
}
void Session::annotate_wall(std::uint8_t x, std::uint8_t y, char dir, bool present) {
    ann_.walls.push_back({x, y, dir, present});
}
void Session::annotate_clear() {
    ann_ = {};
}

}  // namespace mmsim::host
