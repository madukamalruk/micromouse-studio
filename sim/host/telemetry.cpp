#include "sim/host/telemetry.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

namespace mmsim::host {

std::size_t TelemetryChannel::lower_bound(double t0) const {
    std::size_t lo = 0, hi = t.size();
    while (lo < hi) {
        const std::size_t mid = lo + (hi - lo) / 2;
        if (t[mid] < t0)
            lo = mid + 1;
        else
            hi = mid;
    }
    return lo;
}

float TelemetryChannel::value_at(double ts) const {
    // Last sample with time <= ts.
    std::size_t lo = 0, hi = t.size();
    while (lo < hi) {
        const std::size_t mid = lo + (hi - lo) / 2;
        if (t[mid] <= ts)
            lo = mid + 1;
        else
            hi = mid;
    }
    if (lo == 0) return std::numeric_limits<float>::quiet_NaN();
    return v[lo - 1];
}

TelemetryChannel* TelemetryRecorder::add(std::string_view name, double t_s, float value) {
    TelemetryChannel* c = channel(name);
    if (!c) {
        if (ch_.size() >= kMaxChannels) {
            ++dropped_;
            return nullptr;
        }
        index_.emplace(std::string(name), ch_.size());
        ch_.push_back(std::make_unique<TelemetryChannel>(std::string(name), cap_));
        c = ch_.back().get();
    }
    c->t.push(t_s);
    c->v.push(value);
    return c;
}

TelemetryChannel* TelemetryRecorder::channel(std::string_view name) {
    const auto it = index_.find(std::string(name));
    return it == index_.end() ? nullptr : ch_[it->second].get();
}

const TelemetryChannel* TelemetryRecorder::channel(std::string_view name) const {
    const auto it = index_.find(std::string(name));
    return it == index_.end() ? nullptr : ch_[it->second].get();
}

void TelemetryRecorder::clear() {
    ch_.clear();
    index_.clear();
    dropped_ = 0;
}

namespace {
void put_value(std::ostream& out, float v) {
    if (std::isnan(v)) return;
    if (v == 0.0f) v = 0.0f;  // no "-0"
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.7g", static_cast<double>(v));
    out << buf;
}
}  // namespace

std::size_t write_csv(const TelemetryRecorder& rec, std::ostream& out, double t0, double t1,
                      const std::vector<std::string>& channels, std::string_view clock) {
    std::vector<const TelemetryChannel*> cols;
    if (channels.empty()) {
        for (const auto& c : rec.channels()) cols.push_back(c.get());
    } else {
        for (const auto& n : channels)
            if (const auto* c = rec.channel(n)) cols.push_back(c);
    }
    out << "t_s";
    for (const auto* c : cols) out << ',' << c->name;
    out << '\n';
    const TelemetryChannel* clk = rec.channel(clock);
    if (!clk) return 0;
    // One cursor per column: the clock is monotonic, so each column is walked once (O(n)).
    std::vector<std::size_t> cur(cols.size(), 0);
    for (std::size_t k = 0; k < cols.size(); ++k) cur[k] = cols[k]->lower_bound(-1e300);
    std::size_t rows = 0;
    char tbuf[32];
    for (std::size_t i = clk->lower_bound(t0); i < clk->size() && clk->t[i] < t1; ++i) {
        const double ts = clk->t[i];
        std::snprintf(tbuf, sizeof tbuf, "%.3f", ts);
        out << tbuf;
        for (std::size_t k = 0; k < cols.size(); ++k) {
            const TelemetryChannel& c = *cols[k];
            while (cur[k] < c.size() && c.t[cur[k]] <= ts) ++cur[k];
            out << ',';
            if (cur[k] > 0) put_value(out, c.v[cur[k] - 1]);
        }
        out << '\n';
        ++rows;
    }
    return rows;
}

}  // namespace mmsim::host
