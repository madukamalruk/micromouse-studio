#include "sim/host/firmware_host.hpp"

#include "mmhal/hal.h"
#include "sim/hal_sim/host_api.h"
#include "sim/platform/dynlib.hpp"
#include "sim/platform/fiber.hpp"

namespace mmsim::host {

const char* to_string(FwState s) {
    switch (s) {
        case FwState::Unloaded:
            return "unloaded";
        case FwState::LoadError:
            return "load error";
        case FwState::Ready:
            return "ready";
        case FwState::Running:
            return "running";
        case FwState::Halted:
            return "halted";
        case FwState::Faulted:
            return "faulted";
    }
    return "?";
}

// C callbacks handed to the DLL; ctx is the FirmwareHost.
struct HostCallbacks {
    static FirmwareHost& H(void* c) { return *static_cast<FirmwareHost*>(c); }

    static uint32_t millis(void* c) {
        H(c).poll_time();
        return static_cast<uint32_t>(H(c).now_us_ / 1000);
    }
    static uint32_t micros(void* c) {
        H(c).poll_time();
        return static_cast<uint32_t>(H(c).now_us_);
    }
    static void delay_ms(void* c, uint32_t ms) {
        if (ms == 0 && !H(c).in_isr_) return;
        H(c).block_until(H(c).now_us_ + static_cast<std::int64_t>(ms) * 1000, "hal_delay_ms");
    }
    static void wait_tick(void* c) { H(c).block_until(H(c).now_us_ + 1000, "hal_wait_tick"); }
    static void set_systick(void* c, void (*isr)(void)) { H(c).isr_ = isr; }

    static void motor_set_duty(void* c, int side, float duty) {
        H(c).be_.motor_set_duty(side, duty);
    }
    static float battery_voltage(void* c) { return H(c).be_.battery_voltage(); }
    static int32_t encoder_count(void* c, int side) { return H(c).be_.encoder_count(side); }
    static float gyro_z_dps(void* c) { return H(c).be_.gyro_z_dps(); }
    static uint8_t sensor_count(void* c) { return H(c).be_.sensor_count(); }
    static uint16_t sensor_read(void* c, uint8_t ch) { return H(c).be_.sensor_read(ch); }
    static void emitter_set(void* c, uint8_t ch, bool on) { H(c).be_.emitter_set(ch, on); }
    static void led_set(void* c, uint8_t led, bool on) { H(c).be_.led_set(led, on); }
    static bool button_pressed(void* c, uint8_t b) { return H(c).be_.button_pressed(b); }
    static void log(void* c, const char* line) { H(c).be_.log(line ? line : ""); }
    static void telemetry(void* c, const char* ch, float v) { H(c).be_.telemetry(ch ? ch : "", v); }
    static void cell_color(void* c, uint8_t x, uint8_t y, char col) {
        H(c).be_.annotate_cell_color(x, y, col);
    }
    static void cell_text(void* c, uint8_t x, uint8_t y, const char* t) {
        H(c).be_.annotate_cell_text(x, y, t ? t : "");
    }
    static void wall(void* c, uint8_t x, uint8_t y, char d, bool p) {
        H(c).be_.annotate_wall(x, y, d, p);
    }
    static void clear(void* c) { H(c).be_.annotate_clear(); }
};

FirmwareHost::FirmwareHost(HalBackend& backend)
    : be_(backend), api_(std::make_unique<mmsim_host_api>()) {
    mmsim_host_api& a = *api_;
    a.abi = MMSIM_HOST_ABI;
    a.ctx = this;
    a.millis = &HostCallbacks::millis;
    a.micros = &HostCallbacks::micros;
    a.delay_ms = &HostCallbacks::delay_ms;
    a.wait_tick = &HostCallbacks::wait_tick;
    a.set_systick = &HostCallbacks::set_systick;
    a.motor_set_duty = &HostCallbacks::motor_set_duty;
    a.battery_voltage = &HostCallbacks::battery_voltage;
    a.encoder_count = &HostCallbacks::encoder_count;
    a.gyro_z_dps = &HostCallbacks::gyro_z_dps;
    a.sensor_count = &HostCallbacks::sensor_count;
    a.sensor_read = &HostCallbacks::sensor_read;
    a.emitter_set = &HostCallbacks::emitter_set;
    a.led_set = &HostCallbacks::led_set;
    a.button_pressed = &HostCallbacks::button_pressed;
    a.log = &HostCallbacks::log;
    a.telemetry = &HostCallbacks::telemetry;
    a.annotate_cell_color = &HostCallbacks::cell_color;
    a.annotate_cell_text = &HostCallbacks::cell_text;
    a.annotate_wall = &HostCallbacks::wall;
    a.annotate_clear = &HostCallbacks::clear;
}

FirmwareHost::~FirmwareHost() {
    unload();
}

void FirmwareHost::unload() {
    fiber_.reset();  // suspended firmware stack is discarded (C code: nothing to unwind)
    lib_.reset();    // FreeLibrary + delete the private copy
    fw_main_ = nullptr;
    isr_ = nullptr;
    wake_us_ = -1;
    in_isr_ = in_main_ = false;
    polls_ = 0;
    now_us_ = 0;
    if (state_ != FwState::LoadError) state_ = FwState::Unloaded;
}

bool FirmwareHost::load(const std::filesystem::path& dll) {
    unload();
    path_ = dll;
    error_.clear();
    auto fail = [&](std::string msg) {
        unload();
        error_ = std::move(msg);
        state_ = FwState::LoadError;
        return false;
    };
    lib_ = platform::DynamicLibrary::open_copy(dll, error_);
    if (!lib_) return fail(error_);

    auto abi = reinterpret_cast<mmsim_fw_abi_fn>(lib_->symbol("mmsim_fw_abi"));
    auto bind = reinterpret_cast<mmsim_fw_bind_fn>(lib_->symbol("mmsim_fw_bind"));
    fw_main_ = reinterpret_cast<mmsim_fw_main_fn>(lib_->symbol("mmsim_fw_main"));
    if (!abi || !bind || !fw_main_)
        return fail("not a simulator firmware (missing mmsim_fw_* exports): " + dll.string());
    const uint32_t want = (static_cast<uint32_t>(MMSIM_HOST_ABI) << 16) | MMHAL_VERSION;
    if (abi() != want)
        return fail("firmware ABI mismatch: built against host ABI " + std::to_string(abi() >> 16) +
                    " / HAL " + std::to_string(abi() & 0xffff) + ", simulator has " +
                    std::to_string(MMSIM_HOST_ABI) + " / " + std::to_string(MMHAL_VERSION) +
                    "; rebuild it");
    bind(api_.get());
    fiber_ = std::make_unique<platform::Fiber>(&FirmwareHost::fiber_entry, this);
    if (!fiber_->valid()) return fail("cannot create firmware fiber");
    state_ = FwState::Ready;
    return true;
}

bool FirmwareHost::reload() {
    if (path_.empty()) return false;
    const auto p = path_;
    return load(p);
}

void FirmwareHost::fiber_entry(void* self) {
    auto* h = static_cast<FirmwareHost*>(self);
    h->fw_main_();
}

void FirmwareHost::start(std::int64_t now_us) {
    if (state_ != FwState::Ready) return;
    state_ = FwState::Running;
    now_us_ = now_us;
    polls_ = 0;
    in_main_ = true;
    fiber_->resume();
    in_main_ = false;
    if (fiber_->finished() && state_ == FwState::Running) {
        state_ = FwState::Halted;
        be_.log("[host] fw_main returned: firmware halted");
    }
}

void FirmwareHost::tick(std::int64_t now_us) {
    if (state_ != FwState::Running) return;
    now_us_ = now_us;
    if (isr_) {
        in_isr_ = true;
        isr_();
        in_isr_ = false;
        if (state_ != FwState::Running) return;
    }
    if (wake_us_ >= 0 && now_us_ >= wake_us_) {
        wake_us_ = -1;
        polls_ = 0;
        in_main_ = true;
        fiber_->resume();
        in_main_ = false;
        if (fiber_->finished() && state_ == FwState::Running) {
            state_ = FwState::Halted;
            be_.log("[host] fw_main returned: firmware halted");
        }
    }
}

void FirmwareHost::block_until(std::int64_t wake_us, const char* what) {
    if (in_isr_) {
        fault(std::string("blocking HAL call ") + what + "() from the SysTick ISR");
        return;
    }
    if (!in_main_ || platform::Fiber::current() != fiber_.get())
        return;  // not on the firmware fiber
    wake_us_ = wake_us;
    platform::Fiber::yield();
}

void FirmwareHost::poll_time() {
    if (in_isr_ || !in_main_) return;
    if (++polls_ >= kPollYield) block_until(now_us_ + 1000, "poll");
}

void FirmwareHost::fault(std::string why) {
    error_ = std::move(why);
    state_ = FwState::Faulted;
    be_.motor_set_duty(0, 0.0f);
    be_.motor_set_duty(1, 0.0f);
    be_.log(("[host] FAULT: " + error_).c_str());
}

}  // namespace mmsim::host
