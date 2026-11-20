#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "sim/core/maze.hpp"
#include "sim/core/maze_geometry.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/core/sha256.hpp"
#include "sim/core/world.hpp"
#include "sim/host/annotations.hpp"
#include "sim/host/firmware_host.hpp"
#include "sim/host/mms_bridge.hpp"
#include "sim/host/telemetry.hpp"

namespace mmsim::platform {
class ChildProcess;
}

namespace mmsim::host {

struct LogLine {
    std::int64_t time_us;
    std::string text;
};

struct LedEvent {
    std::int64_t time_us;
    int led;
    bool on;
};

/// One simulation run: World (physics) + FirmwareHost (DLL on a fiber) + everything the
/// firmware reports. Implements the non-timing HAL services. Single-threaded.
class Session final : public HalBackend {
  public:
    static constexpr std::size_t kMaxLogLines = 10000;

    Session(core::Maze maze, core::RobotConfig cfg, core::MazeDims dims = {},
            std::uint64_t seed = 1);
    ~Session() override;
    Session(const Session&) = delete;
    Session& operator=(const Session&) = delete;

    bool load(const std::filesystem::path& dll);
    /// mms mode (Slice 7): runs `exe` as a child process speaking the mms text API on
    /// stdin/stdout instead of a firmware DLL. Lockstep: while the bridge is idle each tick
    /// waits (up to the think timeout) for the solver's next command, so solver thinking time
    /// costs no simulated time and runs are deterministic.
    bool load_mms(const std::filesystem::path& exe, MmsOptions opt = {});
    bool mms_mode() const { return mms_ != nullptr; }
    bool mms_running() const;
    std::optional<int> mms_exit_code() const;
    MmsBridge* mms() { return mms_.get(); }
    const MmsBridge* mms() const { return mms_.get(); }
    /// The mms "Reset" button (the solver sees wasReset = true).
    void mms_press_reset();
    void set_mms_think_timeout(std::chrono::milliseconds t) { think_timeout_ = t; }
    /// World back to the start pose + firmware unloaded and reloaded (fresh globals).
    void reset();
    void unload();

    /// Inputs that the firmware reads at boot (e.g. mode buttons).
    void set_button(int btn, bool pressed);

    void step_tick();
    void run_ms(std::uint32_t ms);
    /// Deterministic scrub: go to simulated time t_us. Backwards = reset (fresh firmware,
    /// same seed) + replay; forwards = step. Afterwards the state, recorder and trajectory
    /// hash equal those of an uninterrupted run that stopped at t_us.
    void rewind_to(std::int64_t t_us);

    FwState state() const { return host_.state(); }
    const std::string& error() const { return mms_ ? mms_error_ : host_.error(); }
    const std::filesystem::path& firmware_path() const { return mms_ ? mms_path_ : host_.path(); }
    std::int64_t time_us() const { return world_.time_us(); }

    core::World& world() { return world_; }
    const core::World& world() const { return world_; }

    const std::deque<LogLine>& logs() const { return logs_; }
    const std::vector<LedEvent>& led_events() const { return leds_; }
    bool led(int i) const { return i >= 0 && i < 8 && led_state_[i]; }
    /// Latest value of a channel (firmware or built-in "sim.*").
    std::optional<float> telemetry_value(const std::string& channel) const;
    /// Bounded recorder: built-in channels every tick + firmware hal_telemetry channels.
    const TelemetryRecorder& recorder() const { return rec_; }
    const Annotations& annotations() const { return ann_; }

    /// SHA-256 over every tick's (time, pose, wheel angles) since load/reset: two runs with
    /// equal hashes followed bit-identical trajectories.
    std::string trajectory_hash() const { return traj_.hex(); }
    /// SHA-256 of the loaded firmware DLL file (empty if none).
    const std::string& firmware_hash() const { return fw_hash_; }

  private:
    // HalBackend
    void motor_set_duty(int side, float duty) override;
    float battery_voltage() override;
    std::int32_t encoder_count(int side) override;
    float gyro_z_dps() override;
    std::uint8_t sensor_count() override;
    std::uint16_t sensor_read(std::uint8_t ch) override;
    std::uint16_t sensor_distance(std::uint8_t ch) override;
    void imu(float gyro_dps[3], float accel_mps2[3]) override;
    void emitter_set(std::uint8_t ch, bool on) override;
    void led_set(std::uint8_t led, bool on) override;
    bool button_pressed(std::uint8_t btn) override;
    void log(const char* line) override;
    void telemetry(const char* channel, float value) override;
    void annotate_cell_color(std::uint8_t x, std::uint8_t y, char color) override;
    void annotate_cell_text(std::uint8_t x, std::uint8_t y, const char* text) override;
    void annotate_wall(std::uint8_t x, std::uint8_t y, char dir, bool present) override;
    void annotate_clear() override;

    void clear_outputs();
    void sample();
    bool start_mms();
    void stop_mms();
    void mms_tick();
    void mms_flush();
    void mms_drain_stderr();

    enum Builtin {
        kV,
        kW,
        kX,
        kY,
        kTheta,
        kMotorL,
        kMotorR,
        kWheelL,
        kWheelR,
        kBeta,
        kGyroZ,
        kEncL,
        kEncR,
        kBuiltinCount
    };

    core::World world_;
    FirmwareHost host_;
    bool buttons_[8] = {};
    bool led_state_[8] = {};
    std::deque<LogLine> logs_;
    std::vector<LedEvent> leds_;
    TelemetryRecorder rec_;
    std::array<TelemetryChannel*, kBuiltinCount> builtin_{};
    std::vector<TelemetryChannel*> dist_ch_;
    Annotations ann_;
    core::Sha256 traj_;
    std::string fw_hash_;
    // mms mode
    std::unique_ptr<platform::ChildProcess> proc_;
    std::unique_ptr<MmsBridge> mms_;
    MmsOptions mms_opt_;
    std::filesystem::path mms_path_;
    std::string mms_error_;
    std::chrono::milliseconds think_timeout_{2000};
    bool mms_stalled_ = false;  // solver missed a think timeout: stop blocking until it speaks
    bool mms_exit_logged_ = false;
};

enum class GuardResult { Completed, Hung };

/// Runs session->run_ms(ms) on a worker thread. If it does not finish within wall_timeout
/// (firmware spinning without HAL calls), returns Hung and abandons the worker: the session
/// must not be used again (the shared_ptr keeps it alive for the detached thread).
GuardResult run_guarded(std::shared_ptr<Session> session, std::uint32_t ms,
                        std::chrono::milliseconds wall_timeout);
/// Same guard around arbitrary work on the session (e.g. stepping + sampling a path trace).
/// The work must only touch the session and state it owns (shared_ptr captures).
GuardResult run_guarded(std::shared_ptr<Session> session, std::function<void(Session&)> work,
                        std::chrono::milliseconds wall_timeout);

}  // namespace mmsim::host
