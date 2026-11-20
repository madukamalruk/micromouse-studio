#include "sim/core/analysis/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <numbers>
#include <optional>
#include <set>

namespace mmsim::core::analysis {

namespace {
constexpr int kDx[4] = {0, 1, 0, -1};
constexpr int kDy[4] = {1, 0, -1, 0};

struct V2 {
    double x = 0, y = 0;
};

std::optional<std::pair<double, double>> solve2(V2 p, V2 d, V2 q, V2 e) {
    const double det = d.x * (-e.y) - d.y * (-e.x);
    if (std::abs(det) < 1e-9) return std::nullopt;
    const double rx = q.x - p.x, ry = q.y - p.y;
    return std::make_pair((rx * (-e.y) - ry * (-e.x)) / det, (d.x * ry - d.y * rx) / det);
}

struct Node {
    V2 pos;
    double tin = 0, tout = 0;
    bool turn = false;
    int angle = 0;
    int sign = 0;
    int key = -1;  // corner index, -1 for start/end
    bool diag_from_prev = false;
};

struct Corner {
    bool chain = false;
    int a = 0, b = 0;
};

int py_round(double v) {
    return static_cast<int>(std::nearbyint(v));
}

int mod4(int v) {
    return ((v % 4) + 4) % 4;
}
}  // namespace

double round_half_even(double v, int digits) {
    const double s = std::pow(10.0, digits);
    return std::nearbyint(v * s) / s;
}

std::pair<double, double> straight_time(double length, double v0, double v1, double vmax,
                                        double a) {
    if (length <= 1e-9) return {0.0, std::max(v0, v1)};
    v1 = std::min(v1, std::sqrt(v0 * v0 + 2 * a * length));
    v1 = std::max(v1, std::sqrt(std::max(0.0, v0 * v0 - 2 * a * length)));
    double vp = std::min(vmax, std::sqrt((2 * a * length + v0 * v0 + v1 * v1) / 2.0));
    vp = std::max({vp, v0, v1});
    const double d_acc = (vp * vp - v0 * v0) / (2 * a);
    const double d_dec = (vp * vp - v1 * v1) / (2 * a);
    const double cruise = std::max(0.0, length - d_acc - d_dec);
    return {(vp - v0) / a + (vp - v1) / a + (vp > 0 ? cruise / vp : 0.0), vp};
}

Trajectory compile_trajectory(const std::vector<Cell>& cells, const TrajectoryOptions& o) {
    Trajectory out;
    for (const double x : {o.run_mms, o.accel_mms2, o.radius_mm, o.mu})
        if (!std::isfinite(x) || x <= 0) {
            out.error = "speed, acceleration, radius and friction must be positive numbers";
            return out;
        }
    if (cells.size() < 2) {
        out.error = "the route needs at least two cells";
        return out;
    }
    std::vector<int> dirs;
    for (std::size_t i = 1; i < cells.size(); ++i) {
        const int dx = cells[i].x - cells[i - 1].x, dy = cells[i].y - cells[i - 1].y;
        int d = -1;
        for (int k = 0; k < 4; ++k)
            if (kDx[k] == dx && kDy[k] == dy) d = k;
        if (d < 0) {
            out.error = "consecutive cells must be neighbours";
            return out;
        }
        dirs.push_back(d);
    }
    for (std::size_t i = 1; i < dirs.size(); ++i)
        if (mod4(dirs[i] - dirs[i - 1]) == 2) {
            out.error = "the route turns back on itself (a U-turn is not a primitive)";
            return out;
        }
    const int n = static_cast<int>(dirs.size());
    std::vector<V2> pos;
    for (const Cell c : cells) pos.push_back({(c.x + 0.5) * o.cell_mm, (c.y + 0.5) * o.cell_mm});
    const V2 vec[4] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    std::map<int, int> sign;  // corner index -> +1 left (CCW), -1 right
    for (int i = 1; i < n; ++i)
        if (dirs[static_cast<std::size_t>(i)] != dirs[static_cast<std::size_t>(i - 1)])
            sign[i] =
                mod4(dirs[static_cast<std::size_t>(i)] - dirs[static_cast<std::size_t>(i - 1)]) == 3
                    ? 1
                    : -1;
    const double t90 = o.radius_mm;
    const double t45 = o.radius_mm * std::tan(22.5 * std::numbers::pi / 180.0);
    const double v_curve = std::min(o.run_mms, std::sqrt(o.mu * kGmms2 * o.radius_mm));

    std::set<int> disabled;
    std::vector<Node> nodes;
    std::vector<std::pair<bool, double>> segs;  // (diagonal, length)
    bool found = false;
    const int attempts = static_cast<int>(sign.size()) + 2;
    for (int attempt = 0; attempt < attempts && !found; ++attempt) {
        std::vector<int> ks;
        for (const auto& [k, s] : sign) ks.push_back(k);
        std::vector<Corner> corners;
        std::size_t i = 0;
        while (i < ks.size()) {
            std::size_t j = i;
            if (o.diagonal && !disabled.contains(ks[i]))
                while (j + 1 < ks.size() && ks[j + 1] == ks[j] + 1 &&
                       sign[ks[j + 1]] == -sign[ks[j]])
                    ++j;
            corners.push_back(j > i ? Corner{true, ks[i], ks[j]} : Corner{false, ks[i], ks[i]});
            i = j + 1;
        }
        nodes.clear();
        nodes.push_back({pos[0], 0, 0, false, 0, 0, -1, false});
        bool ok = true;
        for (const Corner& c : corners) {
            if (!c.chain) {
                nodes.push_back({pos[static_cast<std::size_t>(c.a)], t90, t90, true, 90, sign[c.a],
                                 c.a, false});
                continue;
            }
            const V2 e = vec[dirs[static_cast<std::size_t>(c.a - 1)]];
            const V2 m1 = vec[dirs[static_cast<std::size_t>(c.a)]];
            const V2 x = vec[dirs[static_cast<std::size_t>(c.b)]];
            V2 dd{e.x + m1.x, e.y + m1.y};
            const double nrm = std::hypot(dd.x, dd.y);
            dd = {dd.x / nrm, dd.y / nrm};
            const V2 pa0 = pos[static_cast<std::size_t>(c.a)],
                     pb0 = pos[static_cast<std::size_t>(c.b)];
            const V2 mid{(pa0.x + pb0.x) / 2, (pa0.y + pb0.y) / 2};
            const auto sa = solve2(pa0, e, mid, dd);
            const auto sb = solve2(pb0, x, mid, dd);
            if (!sa || !sb || sb->second - sa->second < 2 * t45 + 1e-6) {
                ok = false;
                disabled.insert(c.a);
                break;
            }
            const V2 pa{pa0.x + sa->first * e.x, pa0.y + sa->first * e.y};
            const V2 pb{pb0.x + sb->first * x.x, pb0.y + sb->first * x.y};
            nodes.push_back({pa, t45, t45, true, 45, sign[c.a], c.a, false});
            nodes.push_back({pb, t45, t45, true, 45, sign[c.b], c.b, true});
        }
        if (!ok) continue;
        nodes.push_back({pos.back(), 0, 0, false, 0, 0, -1, false});
        segs.clear();
        for (std::size_t k = 1; k < nodes.size(); ++k) {
            const Node& p = nodes[k - 1];
            const Node& q = nodes[k];
            const double length = std::hypot(q.pos.x - p.pos.x, q.pos.y - p.pos.y) - p.tout - q.tin;
            if (length < -1e-6) {
                ok = false;
                const Corner* hit = nullptr;
                for (const Corner& c : corners)
                    if (c.chain && !hit)
                        for (const int key : {p.key, q.key})
                            if (key >= 0 && c.a <= key && key <= c.b) hit = &c;
                if (!hit) {
                    char buf[96];
                    std::snprintf(buf, sizeof buf, "the route is too tight for a %.0f mm arc",
                                  o.radius_mm);
                    out.error = buf;
                    return out;
                }
                disabled.insert(hit->a);
                break;
            }
            segs.push_back({q.diag_from_prev, std::max(0.0, length)});
        }
        if (ok) found = true;
    }
    if (!found) {
        out.error = "no consistent trajectory found";
        return out;
    }

    std::vector<Primitive> prims;
    for (std::size_t k = 0; k < segs.size(); ++k) {
        Primitive s;
        s.kind = segs[k].first ? "diagonal" : "straight";
        s.length_mm = segs[k].second;
        prims.push_back(s);
        const Node& node = nodes[k + 1];
        if (node.turn) {
            Primitive c;
            c.kind = "curve";
            c.angle_deg = node.angle;
            c.dir = node.sign > 0 ? 'L' : 'R';
            c.radius_mm = o.radius_mm;
            c.length_mm = node.angle * std::numbers::pi / 180.0 * o.radius_mm;
            c.v_mms = v_curve;
            prims.push_back(c);
        }
    }
    double total_mm = 0, total_s = 0, peak_g = 0;
    for (std::size_t k = 0; k < prims.size(); ++k) {
        Primitive& p = prims[k];
        if (p.kind == "curve") {
            const double t = p.length_mm / p.v_mms;
            const double g = p.v_mms * p.v_mms / o.radius_mm / kGmms2;
            peak_g = std::max(peak_g, g);
            p.ms = py_round(t * 1000);
            p.g = round_half_even(g, 3);
        } else {
            const double v0 = k > 0 ? prims[k - 1].v_mms : 0.0;
            const double v1 = k + 1 < prims.size() ? prims[k + 1].v_mms : 0.0;
            const auto [t, vp] = straight_time(p.length_mm, v0, v1, o.run_mms, o.accel_mms2);
            p.v_mms = vp;
            p.ms = py_round(t * 1000);
        }
        if (p.kind == "straight" && p.length_mm < 0.5) continue;
        total_mm += p.length_mm;
        total_s += p.ms / 1000.0;
        out.primitives.push_back(p);
    }
    int seq = 0;
    for (Primitive& p : out.primitives) {
        ++seq;
        char buf[160];
        if (p.kind == "curve") {
            std::snprintf(buf, sizeof buf,
                          "[SEQ %02d] CURVE %d\xC2\xB0%c R=%d mm @ %d mm/s (%d ms)", seq,
                          p.angle_deg, p.dir, py_round(p.radius_mm), py_round(p.v_mms), p.ms);
        } else {
            std::snprintf(buf, sizeof buf, "[SEQ %02d] %s %d mm @ %d mm/s (%d ms)", seq,
                          p.kind == "straight" ? "STRAIGHT" : "DIAGONAL", py_round(p.length_mm),
                          py_round(p.v_mms), p.ms);
        }
        p.text = buf;
        p.seq = seq;
        p.length_mm = round_half_even(p.length_mm, 1);
        p.v_mms = round_half_even(p.v_mms, 1);
        out.diagonals += p.kind == "diagonal";
    }
    out.total_mm = round_half_even(total_mm, 1);
    out.total_s = round_half_even(total_s, 3);
    out.peak_g = round_half_even(peak_g, 3);
    out.limit_mms = round_half_even(std::sqrt(o.mu * kGmms2 * o.radius_mm), 1);
    out.moves = n;
    return out;
}

}  // namespace mmsim::core::analysis
