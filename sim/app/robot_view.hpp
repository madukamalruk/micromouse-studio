#pragma once

#include <vector>

#include "sim/app/camera.hpp"
#include "sim/app/maze_view.hpp"
#include "sim/core/world.hpp"

struct ImDrawList;

namespace mmsim::app {

/// One path-trace sample: axle centre and forward speed there.
struct TracePoint {
    double x, y;
    float speed;  // m/s
};

/// Recent axle-centre positions for the path trace (sampled, bounded), coloured by speed.
class PathTrace {
  public:
    static constexpr std::size_t kMaxPoints = 20000;
    void clear() {
        pts_.clear();
        vmax_ = 0.5f;
    }
    void add(const core::Pose& p, double speed);  // skips points closer than 2 mm
    const std::vector<TracePoint>& points() const { return pts_; }
    float max_speed() const { return vmax_; }  // colour scale (grows, never below 0.5 m/s)

  private:
    std::vector<TracePoint> pts_;
    float vmax_ = 0.5f;
};

/// Speed colour scale of the trace: blue (stopped) -> green -> yellow -> red (max).
Rgba speed_color(float frac);

struct RobotViewOptions {
    bool show_trace = true;
    bool show_crashes = true;
    bool show_beams = true;  // distance-sensor rays of the last sample
};

/// Draws trace, sensor beams, crash markers, body footprint (collision box), wheels and
/// heading arrow.
void draw_robot(ImDrawList* dl, const Camera& cam, const core::World& w, const PathTrace& trace,
                const RobotViewOptions& opt);

/// Outline of the robot at a recorded pose (timeline scrubbing).
void draw_ghost(ImDrawList* dl, const Camera& cam, const core::RobotConfig& cfg,
                const core::Pose& p, Rgba colour = tok::text);

}  // namespace mmsim::app
