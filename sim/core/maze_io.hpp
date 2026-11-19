#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "sim/core/maze.hpp"

namespace mmsim::core {

/// Where and why a maze failed to load. line/col are 1-based; 0 = not applicable.
struct MazeError {
    int line = 0;
    int col = 0;
    std::string message;
    std::string to_string() const;
};

struct MazeLoadResult {
    std::optional<Maze> maze;
    MazeError error;
    bool ok() const { return maze.has_value(); }
    static MazeLoadResult fail(int line, int col, std::string msg) {
        return {std::nullopt, {line, col, std::move(msg)}};
    }
};

/// mms "map" / micromouseonline text format. Cell = 4 columns x 2 rows sharing posts;
/// any non-space character at a wall position is a wall ('+' or 'o' posts both work).
/// 'S' / 'G' inside a cell mark start / goal. CRLF and trailing blank lines tolerated.
MazeLoadResult parse_text_map(std::string_view text);

/// mms "num" format: one line per cell, "X Y N E S W" with 0/1 wall flags.
MazeLoadResult parse_num(std::string_view text);

/// Classic binary .maz: N*N bytes, column-major (index = x*N + y), bits N=1 E=2 S=4 W=8.
MazeLoadResult parse_maz(std::span<const std::uint8_t> bytes);

/// Loads by extension: .maz = binary, .num = num, anything else = text map.
MazeLoadResult load_maze_file(const std::filesystem::path& path);

/// Writers ('o' posts, S/G markers). write_maz requires a square maze.
std::string write_text_map(const Maze& m);
std::string write_num(const Maze& m);
std::vector<std::uint8_t> write_maz(const Maze& m);

}  // namespace mmsim::core
