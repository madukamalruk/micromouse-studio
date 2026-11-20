#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>

#include "sim/core/maze.hpp"

namespace mmsim::host {

/// What the firmware drew on the maze (mms-style): cell colours, cell text and its own wall
/// map. Input is validated so a buggy firmware cannot grow memory without bound: colours must
/// be mms palette chars, cells must lie inside the maze, text is clipped to kMaxText chars,
/// and each wall segment is stored once (south/west map onto the neighbour's north/east).
class Annotations {
  public:
    static constexpr std::size_t kMaxText = 10;
    static constexpr std::string_view kPalette = "kbacgorwyBACGORWY";
    using Cell = std::pair<int, int>;

    Annotations(int width = 16, int height = 16) : w_(width), h_(height) {}

    void set_color(int x, int y, char c);
    void set_text(int x, int y, std::string_view text);   // empty text removes it
    void set_wall(int x, int y, char dir, bool present);  // dir: n/e/s/w (either case)
    void clear_wall(int x, int y, char dir);              // forget (mms clearWall)
    void clear_color(int x, int y) { color.erase({x, y}); }
    void clear_all_color() { color.clear(); }
    void clear_all_text() { text.clear(); }
    void clear();

    /// The firmware's belief about wall (x, y, d); nullopt if it never reported it.
    std::optional<bool> wall(int x, int y, core::Dir d) const;

    std::map<Cell, char> color;
    std::map<Cell, std::string> text;
    /// Canonical segments: key (x, y, 0 = north | 1 = east) -> present.
    std::map<std::tuple<int, int, int>, bool> walls;

  private:
    bool inside(int x, int y) const { return x >= 0 && y >= 0 && x < w_ && y < h_; }
    static std::tuple<int, int, int> key(int x, int y, core::Dir d);
    static bool parse_dir(char c, core::Dir& d);
    int w_, h_;
};

/// mms colour char -> RGB (0xRRGGBB). Returns false for chars outside the palette.
bool mms_color_rgb(char c, std::uint32_t& rgb);

}  // namespace mmsim::host
