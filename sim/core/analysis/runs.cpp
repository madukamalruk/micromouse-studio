#include "sim/core/analysis/runs.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>

#include "sim/core/analysis/trajectory.hpp"

namespace mmsim::core::analysis {

namespace fs = std::filesystem;

RunSeries run_series(const std::vector<double>& t_ms, const std::vector<double>& v,
                     const std::vector<double>& gyro) {
    RunSeries r;
    std::vector<std::size_t> moving;
    for (std::size_t i = 0; i < v.size(); ++i)
        if (std::abs(v[i]) > 20.0) moving.push_back(i);
    if (moving.empty()) {
        r.error = "the robot never moved in this log (no wheel speed above 20 mm/s)";
        return r;
    }
    const std::size_t i0 = moving.front(), i1 = moving.back();
    const double t0 = t_ms[i0];
    double dist = 0.0, prev = t_ms[i0];
    for (std::size_t i = i0; i <= i1; ++i) {
        dist += std::abs(v[i]) * (t_ms[i] - prev) / 1000.0;
        prev = t_ms[i];
        r.t.push_back((t_ms[i] - t0) / 1000.0);
        r.v.push_back(v[i]);
        r.s.push_back(dist);
        r.gyro.push_back(i < gyro.size() ? gyro[i] : 0.0);
    }
    return r;
}

double interp_at(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
    const auto k = static_cast<std::size_t>(std::lower_bound(xs.begin(), xs.end(), x) - xs.begin());
    if (k == 0) return ys.front();
    if (k >= xs.size()) return ys.back();
    const double x0 = xs[k - 1], x1 = xs[k];
    return x1 <= x0 ? ys[k - 1] : ys[k - 1] + (ys[k] - ys[k - 1]) * (x - x0) / (x1 - x0);
}

Comparison compare_runs(const RunSeries& a, const RunSeries& b, double step_mm,
                        std::size_t max_points, double window_mm) {
    Comparison c;
    if (!a.error.empty())
        c.error = "run A: " + a.error;
    else if (!b.error.empty())
        c.error = "run B: " + b.error;
    if (!c.error.empty()) return c;
    const double common = std::min(a.s.back(), b.s.back());
    if (common < 2 * step_mm) {
        char buf[96];
        std::snprintf(buf, sizeof buf, "the two runs share less than %.0f mm of travel",
                      2 * step_mm);
        c.error = buf;
        return c;
    }
    const auto n = static_cast<std::size_t>(std::floor(common / step_mm));
    std::vector<double> ta, tb;
    for (std::size_t k = 0; k <= n; ++k) {
        const double d = static_cast<double>(k) * step_mm;
        c.grid_mm.push_back(round_half_even(d, 1));
        ta.push_back(interp_at(a.s, a.t, d));
        tb.push_back(interp_at(b.s, b.t, d));
        const double va = interp_at(a.s, a.v, d), vb = interp_at(b.s, b.v, d);
        c.v_a.push_back(round_half_even(va, 1));
        c.v_b.push_back(round_half_even(vb, 1));
        c.delta_v_mms.push_back(round_half_even(vb - va, 1));
    }
    std::vector<double> dt(ta.size());
    for (std::size_t k = 0; k < ta.size(); ++k) {
        dt[k] = tb[k] - ta[k];
        c.delta_t_s.push_back(round_half_even(dt[k], 4));
    }
    const std::size_t w =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(window_mm / step_mm)));
    std::pair<double, double> gain{0.0, 0.0}, loss{0.0, 0.0};
    if (dt.size() > w) {
        // min/max of (swing, grid) tuples, like Python's min()/max() on tuples
        std::vector<std::pair<double, double>> sw;
        for (std::size_t k = 0; k + w < dt.size(); ++k)
            sw.push_back({dt[k + w] - dt[k], static_cast<double>(k) * step_mm});
        gain = *std::min_element(sw.begin(), sw.end());
        loss = *std::max_element(sw.begin(), sw.end());
    }
    auto down = [&](const RunSeries& s, std::vector<double>& t, std::vector<double>& v) {
        const std::size_t step =
            std::max<std::size_t>(1, (s.t.size() + max_points - 1) / max_points);
        for (std::size_t i = 0; i < s.t.size(); i += step) {
            t.push_back(round_half_even(s.t[i], 3));
            v.push_back(round_half_even(s.v[i], 1));
        }
    };
    down(a, c.trace_a_t, c.trace_a_v);
    down(b, c.trace_b_t, c.trace_b_v);
    auto summary = [](const RunSeries& s) {
        RunSummary r;
        r.duration_s = round_half_even(s.t.back(), 3);
        r.distance_mm = round_half_even(s.s.back(), 1);
        r.mean_mms = s.t.back() > 0 ? round_half_even(s.s.back() / s.t.back(), 1) : 0.0;
        r.peak_mms = round_half_even(*std::max_element(s.v.begin(), s.v.end()), 1);
        return r;
    };
    c.a = summary(a);
    c.b = summary(b);
    c.common_mm = round_half_even(common, 1);
    const double final_dt = dt.back();
    c.delta_t_final_s = round_half_even(final_dt, 4);
    c.best_gain = {round_half_even(gain.second, 1), round_half_even(std::min(0.0, gain.first), 4),
                   window_mm};
    c.worst_loss = {round_half_even(loss.second, 1), round_half_even(std::max(0.0, loss.first), 4),
                    window_mm};
    char buf[128];
    if (final_dt < 0)
        std::snprintf(buf, sizeof buf, "run B is %.3f s faster over the common %.0f mm", -final_dt,
                      common);
    else
        std::snprintf(buf, sizeof buf, "run B is %.3f s slower over the common %.0f mm", final_dt,
                      common);
    c.verdict = buf;
    return c;
}

Heatmap bin_heatmap(const std::vector<MotionSample>& samples, double cell_mm) {
    Heatmap h;
    if (samples.empty()) {
        h.error = "no samples";
        return h;
    }
    std::map<std::pair<int, int>, HeatCell> cells;  // key (y, x): sorted like the Python output
    for (const MotionSample& s : samples) {
        const int cx = static_cast<int>(std::floor(s.x_mm / cell_mm));
        const int cy = static_cast<int>(std::floor(s.y_mm / cell_mm));
        HeatCell& c = cells[{cy, cx}];
        c.x = cx;
        c.y = cy;
        c.v_max = std::max(c.v_max, std::abs(s.v));
        c.a_long_max = std::max(c.a_long_max, std::abs(s.a_long));
        c.a_lat_max = std::max(c.a_lat_max, std::abs(s.a_lat));
        ++c.samples;
    }
    for (auto& [k, c] : cells) {
        c.v_max = round_half_even(c.v_max, 1);
        c.a_long_max = round_half_even(c.a_long_max, 1);
        c.a_lat_max = round_half_even(c.a_lat_max, 1);
        h.v_max = std::max(h.v_max, c.v_max);
        h.a_long_max = std::max(h.a_long_max, c.a_long_max);
        h.a_lat_max = std::max(h.a_lat_max, c.a_lat_max);
        h.cells.push_back(c);
    }
    return h;
}

Heatmap motion_heatmap(const std::vector<WheelRow>& rows, double counts_per_mm, double cell_mm,
                       double tau_s) {
    if (rows.empty()) return {"the heatmap needs wheel speeds", {}, 0, 0, 0};
    const int seg = rows.back().segment;
    std::vector<const WheelRow*> sel;
    for (const auto& r : rows)
        if (r.segment == seg) sel.push_back(&r);
    double bias_sum = 0;
    int bias_n = 0;
    for (std::size_t i = 0; i < std::min<std::size_t>(200, sel.size()); ++i)
        if (std::abs(sel[i]->wheel_l_cps) < 20 && std::abs(sel[i]->wheel_r_cps) < 20) {
            bias_sum += sel[i]->gyro_dps;
            ++bias_n;
        }
    const double bias = bias_n ? bias_sum / bias_n : 0.0;
    constexpr double kDeg = 3.14159265358979323846 / 180.0;
    double x = cell_mm / 2, y = cell_mm / 2, th = 90.0;
    double prev_t = sel.front()->t_ms, a_lp = 0.0;
    std::optional<double> prev_v;
    std::vector<MotionSample> out;
    for (const WheelRow* r : sel) {
        const double dt = (r->t_ms - prev_t) / 1000.0;
        prev_t = r->t_ms;
        const double w = r->gyro_dps - bias;
        const double v = (r->wheel_l_cps + r->wheel_r_cps) / 2.0 / counts_per_mm;
        if (dt > 0) {
            th += w * dt;
            x += v * std::cos(th * kDeg) * dt;
            y += v * std::sin(th * kDeg) * dt;
            if (prev_v) a_lp += dt / (tau_s + dt) * ((v - *prev_v) / dt - a_lp);
        }
        prev_v = v;
        out.push_back({x, y, v, a_lp, v * w * kDeg});
    }
    return bin_heatmap(out, cell_mm);
}

// ---------------------------------------------------------------- archive

const std::vector<float>* ArchivedRun::column(const std::string& name) const {
    for (std::size_t i = 0; i < channels.size(); ++i)
        if (channels[i] == name && i < data.size()) return &data[i];
    return nullptr;
}

std::string make_run_id(const std::string& ts, const std::string& maze, const std::string& hash) {
    std::string m;
    for (const char ch : maze)
        m += (std::isalnum(static_cast<unsigned char>(ch)) || ch == '-') ? ch : '_';
    if (m.size() > 40) m.resize(40);
    std::string t;  // folder-safe on every OS: "2026-11-20 12:00:00" -> "2026-11-20-12-00-00"
    for (const char ch : ts) t += std::isalnum(static_cast<unsigned char>(ch)) ? ch : '-';
    return t + "_" + (m.empty() ? "maze" : m) + "_" +
           hash.substr(0, std::min<std::size_t>(8, hash.size()));
}

fs::path save_run(const fs::path& root, ArchivedRun run, std::string& err) {
    if (run.id.empty()) {
        err = "run id is empty";
        return {};
    }
    std::size_t n = run.data.empty() ? 0 : run.data[0].size();
    for (const auto& col : run.data)
        if (col.size() != n) {
            err = "telemetry columns differ in length";
            return {};
        }
    std::error_code ec;
    const fs::path dir = root / run.id;
    fs::create_directories(dir, ec);
    if (ec) {
        err = "cannot create " + dir.string() + ": " + ec.message();
        return {};
    }
    nlohmann::json j;
    j["format"] = "mmsim-run-1";
    j["id"] = run.id;
    j["meta"] = run.meta;
    j["numbers"] = run.numbers;
    j["channels"] = run.channels;
    j["samples"] = n;
    {
        std::ofstream f(dir / "run.json", std::ios::binary);
        f << j.dump(1) << '\n';
        if (!f) {
            err = "cannot write run.json";
            return {};
        }
    }
    std::ofstream f(dir / "telemetry.f32", std::ios::binary);
    for (const auto& col : run.data)
        for (const float v : col) {
            std::uint32_t u = 0;
            std::memcpy(&u, &v, 4);
            const char b[4] = {static_cast<char>(u), static_cast<char>(u >> 8),
                               static_cast<char>(u >> 16), static_cast<char>(u >> 24)};
            f.write(b, 4);
        }
    if (!f) {
        err = "cannot write telemetry.f32";
        return {};
    }
    return dir;
}

std::optional<ArchivedRun> load_run(const fs::path& dir, std::string& err) {
    std::ifstream jf(dir / "run.json", std::ios::binary);
    if (!jf) {
        err = "no run.json in " + dir.string();
        return std::nullopt;
    }
    try {
        const auto j = nlohmann::json::parse(jf);
        if (j.value("format", "") != "mmsim-run-1") {
            err = "unknown run format";
            return std::nullopt;
        }
        ArchivedRun r;
        r.id = j.at("id").get<std::string>();
        r.meta = j.at("meta").get<std::map<std::string, std::string>>();
        r.numbers = j.at("numbers").get<std::map<std::string, double>>();
        r.channels = j.at("channels").get<std::vector<std::string>>();
        const auto n = j.at("samples").get<std::size_t>();
        std::ifstream f(dir / "telemetry.f32", std::ios::binary);
        r.data.assign(r.channels.size(), std::vector<float>(n));
        for (auto& col : r.data)
            for (float& v : col) {
                unsigned char b[4];
                if (!f.read(reinterpret_cast<char*>(b), 4)) {
                    err = "telemetry.f32 is truncated";
                    return std::nullopt;
                }
                const std::uint32_t u =
                    b[0] | (b[1] << 8) | (b[2] << 16) | (static_cast<std::uint32_t>(b[3]) << 24);
                std::memcpy(&v, &u, 4);
            }
        return r;
    } catch (const std::exception& e) {
        err = std::string("bad run.json: ") + e.what();
        return std::nullopt;
    }
}

std::vector<fs::path> list_runs(const fs::path& root) {
    std::vector<fs::path> out;
    std::error_code ec;
    if (!fs::is_directory(root, ec)) return out;
    for (const auto& e : fs::directory_iterator(root, ec))
        if (e.is_directory() && fs::exists(e.path() / "run.json")) out.push_back(e.path());
    std::sort(out.begin(), out.end(), [](const fs::path& a, const fs::path& b) {
        return a.filename().string() > b.filename().string();
    });
    return out;
}

}  // namespace mmsim::core::analysis
