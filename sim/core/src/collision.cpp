#include "sim/core/collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mmsim::core {

Obb body_obb(const Pose& p, const BodyConfig& body) {
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    return {{p.x + body.offset_x * c, p.y + body.offset_x * s},
            body.length / 2.0,
            body.width / 2.0,
            p.theta};
}

std::array<Vec2d, 4> obb_corners(const Obb& b) {
    const double c = std::cos(b.theta), s = std::sin(b.theta);
    const Vec2d u{c * b.half_length, s * b.half_length};
    const Vec2d v{-s * b.half_width, c * b.half_width};
    return {
        Vec2d{b.c.x + u.x + v.x, b.c.y + u.y + v.y}, Vec2d{b.c.x - u.x + v.x, b.c.y - u.y + v.y},
        Vec2d{b.c.x - u.x - v.x, b.c.y - u.y - v.y}, Vec2d{b.c.x + u.x - v.x, b.c.y + u.y - v.y}};
}

Contact obb_vs_aabb(const Obb& b, const Rect& r) {
    const double c = std::cos(b.theta), s = std::sin(b.theta);
    const Vec2d rc{(r.x0 + r.x1) / 2.0, (r.y0 + r.y1) / 2.0};
    const double hx = (r.x1 - r.x0) / 2.0, hy = (r.y1 - r.y0) / 2.0;
    const Vec2d axes[4] = {{1.0, 0.0}, {0.0, 1.0}, {c, s}, {-s, c}};

    Contact best;
    best.depth = std::numeric_limits<double>::infinity();
    for (const Vec2d& a : axes) {
        const double ra = b.half_length * std::abs(c * a.x + s * a.y) +
                          b.half_width * std::abs(-s * a.x + c * a.y);
        const double rb = hx * std::abs(a.x) + hy * std::abs(a.y);
        const double d = (b.c.x - rc.x) * a.x + (b.c.y - rc.y) * a.y;
        const double overlap = ra + rb - std::abs(d);
        if (overlap <= 0.0) return {};
        if (overlap < best.depth) {
            best.depth = overlap;
            best.normal = d >= 0.0 ? a : Vec2d{-a.x, -a.y};
        }
    }
    best.hit = true;
    return best;
}

CollisionWorld::CollisionWorld(const Maze& m, const MazeDims& d) {
    for (const Rect& p : post_rects(m, d)) rects_.push_back(p);
    for (const WallRect& w : wall_rects(m, d)) rects_.push_back(w.rect);

    cell_ = d.cell;
    x0_ = -d.cell;
    y0_ = -d.cell;
    gw_ = m.width() + 2;
    gh_ = m.height() + 2;
    grid_.assign(static_cast<std::size_t>(gw_ * gh_), {});
    stamp_.assign(rects_.size(), 0);
    for (int i = 0; i < static_cast<int>(rects_.size()); ++i) {
        const Rect& r = rects_[static_cast<std::size_t>(i)];
        const int ix0 = std::clamp(static_cast<int>(std::floor((r.x0 - x0_) / cell_)), 0, gw_ - 1);
        const int ix1 = std::clamp(static_cast<int>(std::floor((r.x1 - x0_) / cell_)), 0, gw_ - 1);
        const int iy0 = std::clamp(static_cast<int>(std::floor((r.y0 - y0_) / cell_)), 0, gh_ - 1);
        const int iy1 = std::clamp(static_cast<int>(std::floor((r.y1 - y0_) / cell_)), 0, gh_ - 1);
        for (int y = iy0; y <= iy1; ++y)
            for (int x = ix0; x <= ix1; ++x)
                grid_[static_cast<std::size_t>(y * gw_ + x)].push_back(i);
    }
}

template <class Fn>
void CollisionWorld::for_each_candidate(double minx, double miny, double maxx, double maxy,
                                        Fn&& fn) const {
    const int ix0 = std::clamp(static_cast<int>(std::floor((minx - x0_) / cell_)), 0, gw_ - 1);
    const int ix1 = std::clamp(static_cast<int>(std::floor((maxx - x0_) / cell_)), 0, gw_ - 1);
    const int iy0 = std::clamp(static_cast<int>(std::floor((miny - y0_) / cell_)), 0, gh_ - 1);
    const int iy1 = std::clamp(static_cast<int>(std::floor((maxy - y0_) / cell_)), 0, gh_ - 1);
    if (++query_id_ == 0) {
        std::fill(stamp_.begin(), stamp_.end(), 0u);
        query_id_ = 1;
    }
    for (int y = iy0; y <= iy1; ++y) {
        for (int x = ix0; x <= ix1; ++x) {
            for (int i : grid_[static_cast<std::size_t>(y * gw_ + x)]) {
                auto& st = stamp_[static_cast<std::size_t>(i)];
                if (st == query_id_) continue;
                st = query_id_;
                fn(rects_[static_cast<std::size_t>(i)]);
            }
        }
    }
}

Contact CollisionWorld::query(const Obb& b) const {
    if (grid_.empty()) return {};
    double minx = 1e300, maxx = -1e300, miny = 1e300, maxy = -1e300;
    for (const Vec2d& p : obb_corners(b)) {
        minx = std::min(minx, p.x);
        maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y);
        maxy = std::max(maxy, p.y);
    }
    Contact deepest;
    for_each_candidate(minx, miny, maxx, maxy, [&](const Rect& r) {
        const Contact c = obb_vs_aabb(b, r);
        if (c.hit && (!deepest.hit || c.depth > deepest.depth)) deepest = c;
    });
    return deepest;
}

RayHit ray_vs_aabb(Vec2d o, Vec2d d, double tmax, const Rect& r) {
    double t0 = -std::numeric_limits<double>::infinity();
    double t1 = std::numeric_limits<double>::infinity();
    Vec2d n0;
    const double lo[2] = {r.x0, r.y0}, hi[2] = {r.x1, r.y1};
    const double oo[2] = {o.x, o.y}, dd[2] = {d.x, d.y};
    for (int a = 0; a < 2; ++a) {
        if (std::abs(dd[a]) < 1e-15) {
            if (oo[a] < lo[a] || oo[a] > hi[a]) return {};
            continue;
        }
        double ta = (lo[a] - oo[a]) / dd[a], tb = (hi[a] - oo[a]) / dd[a];
        double sign = -1.0;  // entering through the low face: outward normal points to -axis
        if (ta > tb) {
            std::swap(ta, tb);
            sign = 1.0;
        }
        if (ta > t0) {
            t0 = ta;
            n0 = a == 0 ? Vec2d{sign, 0.0} : Vec2d{0.0, sign};
        }
        t1 = std::min(t1, tb);
    }
    if (t0 > t1 || t1 < 0.0) return {};
    if (t0 < 0.0) return {true, 0.0, Vec2d{-d.x, -d.y}};  // origin inside the rectangle
    if (t0 > tmax) return {};
    return {true, t0, n0};
}

RayHit CollisionWorld::raycast(Vec2d o, Vec2d d, double tmax) const {
    RayHit best;
    if (grid_.empty()) return best;
    const Vec2d e{o.x + d.x * tmax, o.y + d.y * tmax};
    for_each_candidate(std::min(o.x, e.x), std::min(o.y, e.y), std::max(o.x, e.x),
                       std::max(o.y, e.y), [&](const Rect& r) {
                           const RayHit h = ray_vs_aabb(o, d, tmax, r);
                           if (h.hit && (!best.hit || h.t < best.t)) best = h;
                       });
    return best;
}

}  // namespace mmsim::core
