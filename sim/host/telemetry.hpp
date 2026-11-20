#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <ostream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace mmsim::host {

/// Fixed-capacity ring buffer: push overwrites the oldest element once full, so memory is
/// bounded however long a run lasts. Index 0 is the oldest element.
template <class T>
class Ring {
  public:
    explicit Ring(std::size_t capacity = 0) : buf_(capacity) {}
    void push(const T& v) {
        if (buf_.empty()) return;
        buf_[(start_ + size_) % buf_.size()] = v;
        if (size_ < buf_.size())
            ++size_;
        else
            start_ = (start_ + 1) % buf_.size();
    }
    std::size_t size() const { return size_; }
    std::size_t capacity() const { return buf_.size(); }
    bool empty() const { return size_ == 0; }
    const T& operator[](std::size_t i) const { return buf_[(start_ + i) % buf_.size()]; }
    const T& back() const { return (*this)[size_ - 1]; }
    void clear() {
        start_ = 0;
        size_ = 0;
    }

  private:
    std::vector<T> buf_;
    std::size_t start_ = 0, size_ = 0;
};

/// One named signal: (time, value) samples, time non-decreasing.
struct TelemetryChannel {
    std::string name;
    Ring<double> t;  // seconds of simulated time
    Ring<float> v;
    TelemetryChannel(std::string n, std::size_t cap) : name(std::move(n)), t(cap), v(cap) {}
    std::size_t size() const { return t.size(); }
    /// Index of the first sample with time >= t0 (size() if none).
    std::size_t lower_bound(double t0) const;
    /// Value at time `ts` (sample-and-hold); NaN before the first sample.
    float value_at(double ts) const;
};

/// All channels of one run. Built-in simulator channels are named "sim.*"; firmware channels
/// (hal_telemetry) keep their own names. Capacity is per channel (default 60 s at 1 kHz) and
/// the number of channels is capped, so total memory is bounded.
class TelemetryRecorder {
  public:
    static constexpr std::size_t kDefaultCapacity = 60000;
    static constexpr std::size_t kMaxChannels = 64;

    explicit TelemetryRecorder(std::size_t capacity_per_channel = kDefaultCapacity)
        : cap_(capacity_per_channel) {}

    /// Appends a sample; creates the channel on first use. Returns nullptr (and drops the
    /// sample) if the channel limit is reached.
    TelemetryChannel* add(std::string_view name, double t_s, float value);
    TelemetryChannel* channel(std::string_view name);
    const TelemetryChannel* channel(std::string_view name) const;
    const std::vector<std::unique_ptr<TelemetryChannel>>& channels() const { return ch_; }
    std::size_t capacity() const { return cap_; }
    std::size_t dropped_channels() const { return dropped_; }
    void clear();  // removes all channels

  private:
    std::size_t cap_;
    std::vector<std::unique_ptr<TelemetryChannel>> ch_;
    std::unordered_map<std::string, std::size_t> index_;
    std::size_t dropped_ = 0;
};

/// Writes CSV: one row per sample of `clock` with time in [t0, t1); every other selected
/// channel is sampled-and-held at that time (empty cell before its first sample or for NaN).
/// Header: t_s,<channel>,... . `channels` empty = all channels. Returns rows written.
std::size_t write_csv(const TelemetryRecorder& rec, std::ostream& out, double t0, double t1,
                      const std::vector<std::string>& channels = {},
                      std::string_view clock = "sim.v");

}  // namespace mmsim::host
