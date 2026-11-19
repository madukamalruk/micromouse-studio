#pragma once

#include <vector>

#include "sim/app/camera.hpp"
#include "sim/core/world.hpp"

struct ImDrawList;

namespace mmsim::app {

/// Recent axle-centre positions for the path trace (sampled, bounded).
class PathTrace {
  public:
    static constexpr std::size_t kMaxPoints = 20000;
    void clear() { pts_.clear(); }
    void add(const core::Pose& p);  // skips points closer than 2 mm to the last one
    const std::vector<core::Vec2d>& points() const { return pts_; }

  private:
    std::vector<core::Vec2d> pts_;
};

struct RobotViewOptions {
    bool show_trace = true;
    bool show_crashes = true;
    bool show_beams = true;  // distance-sensor rays of the last sample
};

/// Draws trace, sensor beams, crash markers, body footprint (collision box), wheels and
/// heading arrow.
void draw_robot(ImDrawList* dl, const Camera& cam, const core::World& w, const PathTrace& trace,
                const RobotViewOptions& opt);

}  // namespace mmsim::app
