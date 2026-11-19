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

Contact CollisionWorld::query(const Obb& b) const {
    if (grid_.empty()) return {};
    double minx = 1e300, maxx = -1e300, miny = 1e300, maxy = -1e300;
    for (const Vec2d& p : obb_corners(b)) {
        minx = std::min(minx, p.x);
        maxx = std::max(maxx, p.x);
        miny = std::min(miny, p.y);
        maxy = std::max(maxy, p.y);
    }
    const int ix0 = std::clamp(static_cast<int>(std::floor((minx - x0_) / cell_)), 0, gw_ - 1);
    const int ix1 = std::clamp(static_cast<int>(std::floor((maxx - x0_) / cell_)), 0, gw_ - 1);
    const int iy0 = std::clamp(static_cast<int>(std::floor((miny - y0_) / cell_)), 0, gh_ - 1);
    const int iy1 = std::clamp(static_cast<int>(std::floor((maxy - y0_) / cell_)), 0, gh_ - 1);

    if (++query_id_ == 0) {
        std::fill(stamp_.begin(), stamp_.end(), 0u);
        query_id_ = 1;
    }
    Contact deepest;
    for (int y = iy0; y <= iy1; ++y) {
        for (int x = ix0; x <= ix1; ++x) {
            for (int i : grid_[static_cast<std::size_t>(y * gw_ + x)]) {
                auto& st = stamp_[static_cast<std::size_t>(i)];
                if (st == query_id_) continue;
                st = query_id_;
                const Contact c = obb_vs_aabb(b, rects_[static_cast<std::size_t>(i)]);
                if (c.hit && (!deepest.hit || c.depth > deepest.depth)) deepest = c;
            }
        }
    }
    return deepest;
}

}  // namespace mmsim::core
