#include "sim/host/session.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <utility>

#include "sim/platform/process.hpp"

namespace mmsim::host {

Session::Session(core::Maze maze, core::RobotConfig cfg, core::MazeDims dims, std::uint64_t seed)
    : world_(std::move(maze), std::move(cfg), dims, seed),
      host_(*this),
      ann_(world_.maze().width(), world_.maze().height()) {}

Session::~Session() = default;

bool Session::load(const std::filesystem::path& dll) {
    stop_mms();
    world_.reset();
    clear_outputs();
    fw_hash_.clear();
    std::ifstream in(dll, std::ios::binary);
    if (in) {
        const std::string bytes((std::istreambuf_iterator<char>(in)), {});
        fw_hash_ = core::sha256_hex(bytes);
    }
    return host_.load(dll);
}

void Session::reset() {
    world_.reset();
    clear_outputs();
    if (mms_) {
        start_mms();  // fresh solver process, same maze
        return;
    }
    host_.reload();
}

void Session::unload() {
    stop_mms();
    host_.unload();
    world_.reset();
    clear_outputs();
}

void Session::clear_outputs() {
    logs_.clear();
    leds_.clear();
    rec_.clear();
    builtin_.fill(nullptr);
    dist_ch_.clear();
    ann_.clear();
    traj_ = core::Sha256();
    for (bool& l : led_state_) l = false;
}

void Session::set_button(int btn, bool pressed) {
    if (btn >= 0 && btn < 8) buttons_[btn] = pressed;
}

void Session::step_tick() {
    if (mms_) {
        mms_tick();
        return;
    }
    if (host_.state() == FwState::Ready) host_.start(world_.time_us());
    world_.step_tick();
    host_.tick(world_.time_us());
    sample();
}

// --- mms mode -------------------------------------------------------------------------

bool Session::load_mms(const std::filesystem::path& exe, MmsOptions opt) {
    host_.unload();
    world_.reset();
    clear_outputs();
    mms_opt_ = opt;
    mms_path_ = exe;
    fw_hash_.clear();
    std::ifstream in(exe, std::ios::binary);
    if (in) {
        const std::string bytes((std::istreambuf_iterator<char>(in)), {});
        fw_hash_ = core::sha256_hex(bytes);
    }
    return start_mms();
}

bool Session::start_mms() {
    if (proc_) proc_->kill();
    proc_ = std::make_unique<platform::ChildProcess>();
    mms_ = std::make_unique<MmsBridge>(world_, ann_, mms_opt_);
    mms_stalled_ = false;
    mms_exit_logged_ = false;
    mms_error_.clear();
    if (!proc_->start(mms_path_, {}, {}, mms_error_)) {
        log(("[mms] " + mms_error_).c_str());
        return false;
    }
    log(("[mms] started " + mms_path_.filename().string()).c_str());
    return true;
}

void Session::stop_mms() {
    if (proc_) proc_->kill();
    proc_.reset();
    mms_.reset();
    mms_error_.clear();
}

bool Session::mms_running() const {
    return proc_ && proc_->running();
}

std::optional<int> Session::mms_exit_code() const {
    return proc_ ? proc_->exit_code() : std::nullopt;
}

void Session::mms_press_reset() {
    if (mms_) mms_->press_reset();
}

void Session::mms_flush() {
    for (const std::string& r : mms_->take_responses())
        if (proc_) proc_->write(r + "\n");
}

void Session::mms_drain_stderr() {
    if (!proc_) return;
    for (const std::string& l : proc_->take_stderr_lines()) log(("[solver] " + l).c_str());
}

void Session::mms_tick() {
    // Everything the solver already sent, without waiting.
    while (proc_) {
        auto l = proc_->read_line(std::chrono::milliseconds(0));
        if (!l) break;
        mms_stalled_ = false;
        mms_->feed_line(*l);
    }
    mms_->process();
    mms_flush();
    // Lockstep: an idle mouse waits for the solver's next command (no sim time passes).
    while (proc_ && mms_->idle() && !proc_->eof() && !mms_stalled_) {
        auto l = proc_->read_line(think_timeout_);
        if (!l) {
            if (!proc_->eof()) {
                mms_stalled_ = true;
                log("[mms] solver silent for the think timeout: simulation continues");
            }
            break;
        }
        mms_->feed_line(*l);
        mms_->process();
        mms_flush();
    }
    if (proc_ && proc_->eof() && !mms_exit_logged_) {
        proc_->wait(std::chrono::milliseconds(2000));
        mms_drain_stderr();
        const auto code = proc_->exit_code();
        log(("[mms] solver exited, code " + (code ? std::to_string(*code) : std::string("?")))
                .c_str());
        mms_exit_logged_ = true;
    }
    mms_drain_stderr();
    mms_->tick();
    mms_flush();
    sample();
}

void Session::run_ms(std::uint32_t ms) {
    for (std::uint32_t i = 0; i < ms; ++i) step_tick();
}

void Session::rewind_to(std::int64_t t_us) {
    if (t_us < world_.time_us()) reset();
    while (world_.time_us() < t_us) step_tick();
}

void Session::sample() {
    const double t = static_cast<double>(world_.time_us()) * 1e-6;
    if (!builtin_[kV]) {
        static constexpr const char* kNames[kBuiltinCount] = {
            "sim.v",       "sim.w",       "sim.x",       "sim.y",       "sim.theta",
            "sim.motor_l", "sim.motor_r", "sim.wheel_l", "sim.wheel_r", "sim.beta",
            "sim.gyro_z",  "sim.enc_l",   "sim.enc_r"};
        // Create the channels once (add() needs a sample), then empty them so every
        // built-in channel holds exactly one sample per tick.
        const auto make = [&](const std::string& name) {
            TelemetryChannel* c = rec_.add(name, t, 0.0f);
            if (c) {
                c->t.clear();
                c->v.clear();
            }
            return c;
        };
        for (int i = 0; i < kBuiltinCount; ++i) builtin_[i] = make(kNames[i]);
        for (std::size_t ch = 0; ch < world_.sensor_count(); ++ch)
            dist_ch_.push_back(make("sim.d." + world_.sensors().config(ch).name));
    }
    const core::DriveState& d = world_.drive();
    constexpr double kDeg = 180.0 / 3.14159265358979323846;
    const auto put = [t](TelemetryChannel* c, double v) {
        if (!c) return;
        c->t.push(t);
        c->v.push(static_cast<float>(v));
    };
    put(builtin_[kV], d.v * 1000.0);
    put(builtin_[kW], d.w * kDeg);
    put(builtin_[kX], d.pose.x);
    put(builtin_[kY], d.pose.y);
    put(builtin_[kTheta], d.pose.theta * kDeg);
    put(builtin_[kMotorL], world_.motor_voltage(0));
    put(builtin_[kMotorR], world_.motor_voltage(1));
    put(builtin_[kWheelL], d.wheel_speed[0]);
    put(builtin_[kWheelR], d.wheel_speed[1]);
    put(builtin_[kBeta], d.beta * kDeg);
    put(builtin_[kGyroZ], world_.imu().gyro_dps[2]);
    put(builtin_[kEncL], world_.encoder_count(0));
    put(builtin_[kEncR], world_.encoder_count(1));
    for (std::size_t ch = 0; ch < dist_ch_.size(); ++ch) {
        const std::uint16_t mm = world_.sensor_distance_mm(static_cast<int>(ch));
        put(dist_ch_[ch], mm == core::kDistanceNone ? std::numeric_limits<double>::quiet_NaN()
                                                    : static_cast<double>(mm));
    }
    // Trajectory hash: the exact bits of the state that define where the robot went.
    const std::int64_t now = world_.time_us();
    const double st[6] = {d.pose.x,         d.pose.y,         d.pose.theta,
                          d.wheel_angle[0], d.wheel_angle[1], d.beta};
    traj_.update(&now, sizeof now);
    traj_.update(st, sizeof st);
}

std::optional<float> Session::telemetry_value(const std::string& channel) const {
    const TelemetryChannel* c = rec_.channel(channel);
    if (!c || c->size() == 0) return std::nullopt;
    return c->v.back();
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
    if (!channel) return;
    rec_.add(channel, static_cast<double>(world_.time_us()) * 1e-6, value);
}

void Session::annotate_cell_color(std::uint8_t x, std::uint8_t y, char color) {
    ann_.set_color(x, y, color);
}
void Session::annotate_cell_text(std::uint8_t x, std::uint8_t y, const char* text) {
    ann_.set_text(x, y, text ? text : "");
}
void Session::annotate_wall(std::uint8_t x, std::uint8_t y, char dir, bool present) {
    ann_.set_wall(x, y, dir, present);
}
void Session::annotate_clear() {
    ann_.clear();
}

}  // namespace mmsim::host
