#include "sim/app/robot_view.hpp"

#include <imgui.h>

#include <algorithm>
#include <cmath>

#include "sim/app/maze_view.hpp"

namespace mmsim::app {

namespace {

constexpr Rgba kBody{70, 140, 220, 210};
constexpr Rgba kBodyEdge{150, 200, 255, 255};
constexpr Rgba kBodyContact{255, 170, 60, 255};
constexpr Rgba kWheel{20, 22, 26, 255};
constexpr Rgba kArrow{240, 245, 255, 255};
constexpr Rgba kTrace{90, 200, 140, 200};
constexpr Rgba kCrash{255, 90, 60, 255};
constexpr Rgba kBeamIr{255, 80, 120, 255};
constexpr Rgba kBeamTof{80, 220, 255, 255};
constexpr Rgba kBeamOff{150, 150, 160, 255};

ImVec2 S(const Camera& cam, double x, double y) {
    const Vec2 p = cam.world_to_screen({x, y});
    return {static_cast<float>(p.x), static_cast<float>(p.y)};
}

// Rectangle in robot frame (forward = +x, left = +y) -> screen quad.
void robot_rect(ImDrawList* dl, const Camera& cam, const core::Pose& p, double cx, double cy,
                double hl, double hw, Rgba fill, Rgba edge, float thick) {
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    auto pt = [&](double lx, double ly) {
        const double fx = cx + lx, fy = cy + ly;
        return S(cam, p.x + fx * c - fy * s, p.y + fx * s + fy * c);
    };
    const ImVec2 q[4] = {pt(hl, hw), pt(-hl, hw), pt(-hl, -hw), pt(hl, -hw)};
    dl->AddQuadFilled(q[0], q[1], q[2], q[3], fill.packed());
    if (edge.a) dl->AddQuad(q[0], q[1], q[2], q[3], edge.packed(), thick);
}

}  // namespace

void PathTrace::add(const core::Pose& p) {
    if (!pts_.empty()) {
        const auto& b = pts_.back();
        if (std::hypot(p.x - b.x, p.y - b.y) < 0.002) return;
    }
    if (pts_.size() >= kMaxPoints) pts_.erase(pts_.begin(), pts_.begin() + kMaxPoints / 4);
    pts_.push_back({p.x, p.y});
}

void draw_robot(ImDrawList* dl, const Camera& cam, const core::World& w, const PathTrace& trace,
                const RobotViewOptions& opt) {
    const float ppm = static_cast<float>(cam.pixels_per_metre());
    const auto& cfg = w.config();
    const core::Pose& p = w.drive().pose;

    if (opt.show_trace && trace.points().size() >= 2) {
        std::vector<ImVec2> pts;
        pts.reserve(trace.points().size() + 1);
        for (const auto& q : trace.points()) pts.push_back(S(cam, q.x, q.y));
        pts.push_back(S(cam, p.x, p.y));
        dl->AddPolyline(pts.data(), static_cast<int>(pts.size()), kTrace.packed(), ImDrawFlags_None,
                        std::max(1.0f, 0.002f * ppm));
    }

    // Sensor beams (behind the body): every ray of the last sample, faded by its weight in
    // the beam profile; a dot where it hits. Emitter off -> grey.
    if (opt.show_beams) {
        const auto& ss = w.sensors();
        const float thick = std::max(1.0f, 0.0008f * ppm);
        const float dot = std::max(2.0f, 0.0018f * ppm);
        for (std::size_t ch = 0; ch < ss.size(); ++ch) {
            Rgba col = ss.config(ch).type == core::SensorType::ToF ? kBeamTof : kBeamIr;
            if (ss.config(ch).type == core::SensorType::IR && !w.emitter(static_cast<int>(ch)))
                col = kBeamOff;
            for (const auto& r : ss.beam(ch)) {
                Rgba c = col;
                c.a = static_cast<std::uint8_t>(40 + 170 * std::clamp(r.weight, 0.0, 1.0));
                dl->AddLine(S(cam, r.from.x, r.from.y), S(cam, r.to.x, r.to.y), c.packed(), thick);
                if (r.hit) dl->AddCircleFilled(S(cam, r.to.x, r.to.y), dot, c.packed());
            }
        }
    }

    // Body = exactly the collision box.
    const auto& b = cfg.body;
    const float edge = std::max(1.0f, 0.0012f * ppm);
    robot_rect(dl, cam, p, b.offset_x, 0.0, b.length / 2, b.width / 2, kBody,
               w.in_contact() ? kBodyContact : kBodyEdge, edge);

    // Wheels: diameter 2r, width 6 mm, at +-track/2.
    const double hw_track = cfg.track_width / 2.0;
    for (double side : {1.0, -1.0})
        robot_rect(dl, cam, p, 0.0, side * hw_track, cfg.wheel_radius, 0.003, kWheel, {0, 0, 0, 0},
                   0.0f);

    // Heading arrow from the axle centre.
    const double L = b.offset_x + b.length / 2.0 - 0.008;
    const double c = std::cos(p.theta), s = std::sin(p.theta);
    const ImVec2 a0 = S(cam, p.x, p.y), a1 = S(cam, p.x + L * c, p.y + L * s);
    dl->AddLine(a0, a1, kArrow.packed(), std::max(1.5f, 0.0025f * ppm));
    const double h = 0.012;
    dl->AddTriangleFilled(
        a1, S(cam, p.x + (L - h) * c - 0.6 * h * s, p.y + (L - h) * s + 0.6 * h * c),
        S(cam, p.x + (L - h) * c + 0.6 * h * s, p.y + (L - h) * s - 0.6 * h * c), kArrow.packed());
    dl->AddCircleFilled(a0, std::max(2.0f, 0.003f * ppm), kArrow.packed());

    if (opt.show_crashes) {
        for (const auto& e : w.crashes()) {
            const double fx = e.pose.x + (b.offset_x + b.length / 2) * std::cos(e.pose.theta);
            const double fy = e.pose.y + (b.offset_x + b.length / 2) * std::sin(e.pose.theta);
            const ImVec2 m = S(cam, fx, fy);
            const float r = std::max(4.0f, 0.008f * ppm);
            dl->AddLine({m.x - r, m.y - r}, {m.x + r, m.y + r}, kCrash.packed(), 2.0f);
            dl->AddLine({m.x - r, m.y + r}, {m.x + r, m.y - r}, kCrash.packed(), 2.0f);
        }
    }
}

}  // namespace mmsim::app
