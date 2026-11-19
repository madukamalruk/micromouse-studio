#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>

namespace mmsim::platform {
class DynamicLibrary;
class Fiber;
}  // namespace mmsim::platform

struct mmsim_host_api;

namespace mmsim::host {

enum class FwState {
    Unloaded,   // no firmware
    LoadError,  // load failed; see error()
    Ready,      // loaded, fw_main not started yet
    Running,    // lockstep active
    Halted,     // fw_main returned
    Faulted,    // illegal HAL use (e.g. blocking call in the SysTick ISR); see error()
};

const char* to_string(FwState s);

/// Non-timing HAL services, implemented by the Session (world, LEDs, logs, telemetry).
class HalBackend {
  public:
    virtual ~HalBackend() = default;
    virtual void motor_set_duty(int side, float duty) = 0;
    virtual float battery_voltage() = 0;
    virtual std::int32_t encoder_count(int side) = 0;
    virtual float gyro_z_dps() = 0;
    virtual std::uint8_t sensor_count() = 0;
    virtual std::uint16_t sensor_read(std::uint8_t ch) = 0;
    virtual std::uint16_t sensor_distance(std::uint8_t ch) = 0;
    /// Measured IMU sample: body-frame rates (deg/s) and accelerations (m/s^2).
    virtual void imu(float gyro_dps[3], float accel_mps2[3]) = 0;
    virtual void emitter_set(std::uint8_t ch, bool on) = 0;
    virtual void led_set(std::uint8_t led, bool on) = 0;
    virtual bool button_pressed(std::uint8_t btn) = 0;
    virtual void log(const char* line) = 0;
    virtual void telemetry(const char* channel, float value) = 0;
    virtual void annotate_cell_color(std::uint8_t x, std::uint8_t y, char color) = 0;
    virtual void annotate_cell_text(std::uint8_t x, std::uint8_t y, const char* text) = 0;
    virtual void annotate_wall(std::uint8_t x, std::uint8_t y, char dir, bool present) = 0;
    virtual void annotate_clear() = 0;
};

/// Loads a firmware DLL and runs its fw_main() on a fiber in lockstep with simulated time
/// (docs/decisions.md D2, D6):
///   start(t0)  -> fw_main runs until its first blocking HAL call
///   tick(now)  -> SysTick ISR (on the caller's stack), then main resumes if its wait ended
/// Blocking calls: hal_delay_ms, hal_wait_tick. Time never advances while firmware runs, so
/// a busy-wait on hal_millis()/hal_micros() yields one tick after kPollYield reads.
class FirmwareHost {
  public:
    static constexpr int kPollYield = 256;

    explicit FirmwareHost(HalBackend& backend);
    ~FirmwareHost();
    FirmwareHost(const FirmwareHost&) = delete;
    FirmwareHost& operator=(const FirmwareHost&) = delete;

    bool load(const std::filesystem::path& dll);
    /// Unload + load the same file: all firmware globals start fresh.
    bool reload();
    void unload();

    void start(std::int64_t now_us);
    void tick(std::int64_t now_us);

    FwState state() const { return state_; }
    const std::string& error() const { return error_; }
    const std::filesystem::path& path() const { return path_; }

  private:
    friend struct HostCallbacks;

    static void fiber_entry(void* self);
    void block_until(std::int64_t wake_us, const char* what);
    void poll_time();
    void fault(std::string why);

    HalBackend& be_;
    std::unique_ptr<mmsim_host_api> api_;
    std::unique_ptr<platform::DynamicLibrary> lib_;
    std::unique_ptr<platform::Fiber> fiber_;
    void (*fw_main_)(void) = nullptr;
    void (*isr_)(void) = nullptr;

    std::filesystem::path path_;
    FwState state_ = FwState::Unloaded;
    std::string error_;
    std::int64_t now_us_ = 0;
    std::int64_t wake_us_ = -1;  // -1 = main is not waiting
    bool in_isr_ = false;
    bool in_main_ = false;
    int polls_ = 0;
};

}  // namespace mmsim::host
