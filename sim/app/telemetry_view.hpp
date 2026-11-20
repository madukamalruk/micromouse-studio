#pragma once

#include <set>
#include <string>
#include <vector>

#include "sim/host/telemetry.hpp"

namespace mmsim::app {

/// GUI state of the Telemetry window (ImPlot). While the sim runs the x axes follow a
/// rolling window ending at "now"; when paused they are free (zoom/pan with the mouse) and a
/// scrub cursor selects a recorded instant (ghost robot on the maze, "rewind to cursor").
struct TelemetryView {
    double window_s = 5.0;  // rolling window length while following
    bool follow = true;     // x axes track "now" (forced on while running)
    double view_t0 = 0.0;   // visible x range of the last frame (CSV export of the view)
    double view_t1 = 0.0;
    bool scrub = false;                // cursor active (paused)
    bool jump = false;                 // timeline moved: centre the view on the cursor once
    double cursor_t = 0.0;             // seconds
    std::set<std::string> custom;      // channels shown in the "Custom" plot
    std::vector<std::string> plotted;  // every channel drawn this frame (for CSV export)
};

struct TelemetryActions {
    bool export_view = false;  // write the visible window to CSV
    bool export_all = false;   // write everything recorded
    bool rewind = false;       // rewind the simulation to cursor_t
    bool live = false;         // leave scrub mode
};

/// Draws the Telemetry window contents (call between ImGui::Begin/End).
TelemetryActions draw_telemetry(TelemetryView& v, const host::TelemetryRecorder& rec, double now_s,
                                bool running);

/// Index range [i0, i1) of samples to draw for the visible x range [x0, x1] (one sample of
/// margin on each side so lines reach the plot edges).
void visible_range(const host::TelemetryChannel& c, double x0, double x1, std::size_t& i0,
                   std::size_t& i1);

}  // namespace mmsim::app
