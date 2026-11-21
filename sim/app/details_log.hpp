#pragma once
// "Move & Firmware Details": the structured, timestamped run log (v0.2.0), in the spirit of the
// original mms details panel. No GUI dependencies; the panel only draws what is here.
//
//   Moves     cell transitions "(x,y) N", turns "Turn Right 90 deg at (2,4)", straight runs,
//             GOAL REACHED / speed run milestones and wall crashes. Derived from the robot's
//             pose (MoveTracker), so they are the same for firmware and for mms solvers.
//   Firmware  the raw hal_log(...) lines and, for mms solvers, their stderr lines.
//   Errors    ERROR log lines and wall crashes.

#include <cstddef>
#include <deque>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mmsim::app {

enum DetailFlag : unsigned {
    kDetMove = 1,
    kDetFirmware = 2,
    kDetError = 4,
    kDetMilestone = 8,
};

enum class DetailFilter { All, Moves, Firmware, Errors };

struct DetailEntry {
    double t_s = 0.0;
    unsigned flags = 0;
    std::string text;
};

/// Short fixed-width tag: "RUN " (milestone), "ERR ", "MOVE", "FW  ".
const char* detail_tag(const DetailEntry& e);
/// One log row as copied to the clipboard: "   12.345  MOVE  Cell (3,4) N".
std::string format_detail(const DetailEntry& e);
/// Case-insensitive substring search over the tag and text; `filter` picks the category.
bool detail_matches(const DetailEntry& e, DetailFilter filter, std::string_view search);

/// A pose sample as the simulator records it: metres, radians (CCW from +x).
struct PoseSample {
    double t_s = 0.0;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

/// Derives motion events from a stream of pose samples (feed every recorded tick, in order).
class MoveTracker {
  public:
    explicit MoveTracker(double cell_m = 0.18) : cell_(cell_m) {}
    void reset();
    /// Appends the events this sample completes (zero or more) to `out`.
    void feed(const PoseSample& s, std::vector<DetailEntry>& out);
    double last_t() const { return last_t_; }
    bool started() const { return started_; }
    /// The cell the robot was in at time t (the oldest known one for earlier times).
    std::pair<int, int> cell_at(double t_s) const;

  private:
    struct CellStamp {
        double t;
        int x, y;
    };
    void flush_straight(double t, std::vector<DetailEntry>& out);

    double cell_;
    bool started_ = false;
    double last_t_ = -1.0;
    double px_ = 0.0, py_ = 0.0;
    double theta_u_ = 0.0, prev_theta_ = 0.0;
    int ref_q_ = 0;  // settled heading as counter-clockwise quarter turns
    double near_since_ = -1.0;
    double still_since_ = -1.0;
    int cx_ = 0, cy_ = 0;
    int run_cells_ = 0;
    double run_last_t_ = 0.0;
    std::vector<CellStamp> history_;
};

/// The entries shown in the panel, newest last, bounded.
class DetailsLog {
  public:
    static constexpr std::size_t kMax = 5000;

    void add(DetailEntry e);
    /// Raw firmware / solver line: always a Firmware entry; ERROR lines also Error; milestone
    /// lines (goal reached, speed run ...) also add a readable Move entry.
    void ingest_log(double t_s, const std::string& text, std::vector<DetailEntry>& out);
    /// A wall hit: a Move + Error entry.
    static DetailEntry crash_entry(double t_s, double speed_mps, int cx, int cy);

    void clear() { entries_.clear(); }
    const std::deque<DetailEntry>& entries() const { return entries_; }

    /// The rows the filter and search keep, as plain text (for the Copy button).
    std::string to_text(DetailFilter filter, std::string_view search) const;

  private:
    std::deque<DetailEntry> entries_;
};

}  // namespace mmsim::app
