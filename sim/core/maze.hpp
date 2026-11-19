#pragma once

#include <cstdint>
#include <vector>

namespace mmsim::core {

/// Compass direction. Values match bit positions of the classic .maz encoding.
enum class Dir : std::uint8_t { N = 0, E = 1, S = 2, W = 3 };

constexpr std::uint8_t wall_bit(Dir d) { return static_cast<std::uint8_t>(1u << static_cast<unsigned>(d)); }
constexpr Dir opposite(Dir d) { return static_cast<Dir>((static_cast<unsigned>(d) + 2u) & 3u); }
constexpr int dx(Dir d) { return d == Dir::E ? 1 : d == Dir::W ? -1 : 0; }
constexpr int dy(Dir d) { return d == Dir::N ? 1 : d == Dir::S ? -1 : 0; }
inline constexpr Dir kAllDirs[4] = {Dir::N, Dir::E, Dir::S, Dir::W};

/// Cell coordinate. (0,0) is bottom-left (south-west); x grows east, y grows north.
struct Cell {
    int x = 0;
    int y = 0;
    friend bool operator==(Cell, Cell) = default;
};

/// Rectangular micromouse maze: per-cell wall bitmask (N=1 E=2 S=4 W=8), start, goals.
/// Shared walls are always stored on both neighbouring cells (consistent by construction).
class Maze {
public:
    Maze() = default;
    Maze(int width, int height);

    int width() const { return width_; }
    int height() const { return height_; }
    bool in_bounds(int x, int y) const { return x >= 0 && y >= 0 && x < width_ && y < height_; }
    bool in_bounds(Cell c) const { return in_bounds(c.x, c.y); }

    std::uint8_t walls(int x, int y) const { return walls_[index(x, y)]; }
    bool has_wall(int x, int y, Dir d) const { return (walls(x, y) & wall_bit(d)) != 0; }
    bool has_wall(Cell c, Dir d) const { return has_wall(c.x, c.y, d); }

    /// Sets/clears a wall on (x,y) and the matching wall of its neighbour (if any).
    void set_wall(int x, int y, Dir d, bool present);
    /// Adds walls around the whole perimeter.
    void add_boundary();
    /// True if every perimeter wall is present.
    bool is_enclosed() const;
    /// Number of distinct wall segments (a shared wall counts once).
    int wall_count() const;

    Cell start() const { return start_; }
    Dir start_heading() const { return start_heading_; }
    const std::vector<Cell>& goals() const { return goals_; }
    bool is_goal(Cell c) const;

    void set_start(Cell c, Dir heading) { start_ = c; start_heading_ = heading; }
    /// Start heading from walls: the single open side if exactly one, else North.
    void infer_start_heading();
    /// Stores goals in canonical order (y, then x), duplicates removed.
    void set_goals(std::vector<Cell> goals);
    /// Centre 2x2 (even sizes), centre 1x2/2x1/1x1 otherwise.
    void set_default_goals();

    /// FNV-1a 64 over [width, height, walls row-major from (0,0)]. Start/goals excluded.
    std::uint64_t hash() const;

    friend bool operator==(const Maze&, const Maze&) = default;

private:
    std::size_t index(int x, int y) const {
        return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
    }

    int width_ = 0;
    int height_ = 0;
    std::vector<std::uint8_t> walls_;
    Cell start_{};
    Dir start_heading_ = Dir::N;
    std::vector<Cell> goals_;
};

}  // namespace mmsim::core
