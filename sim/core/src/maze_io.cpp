#include "sim/core/maze_io.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>

namespace mmsim::core {

std::string MazeError::to_string() const {
    std::ostringstream os;
    if (line > 0) {
        os << "line " << line;
        if (col > 0) os << ", col " << col;
        os << ": ";
    }
    os << message;
    return os.str();
}

namespace {

constexpr int kMaxDim = 255;

std::vector<std::string_view> split_lines(std::string_view text) {
    std::vector<std::string_view> lines;
    std::size_t pos = 0;
    while (pos <= text.size()) {
        std::size_t nl = text.find('\n', pos);
        std::string_view line =
            text.substr(pos, nl == std::string_view::npos ? std::string_view::npos : nl - pos);
        if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
        lines.push_back(line);
        if (nl == std::string_view::npos) break;
        pos = nl + 1;
    }
    return lines;
}

bool is_blank(std::string_view s) {
    return std::all_of(s.begin(), s.end(), [](char c) { return c == ' ' || c == '\t'; });
}

/// Enclosure check that reports the first missing boundary wall.
std::optional<std::string> enclosure_error(const Maze& m) {
    for (int x = 0; x < m.width(); ++x) {
        if (!m.has_wall(x, 0, Dir::S))
            return "maze is not enclosed: missing south wall of cell (" + std::to_string(x) + ",0)";
        if (!m.has_wall(x, m.height() - 1, Dir::N))
            return "maze is not enclosed: missing north wall of cell (" + std::to_string(x) + "," +
                   std::to_string(m.height() - 1) + ")";
    }
    for (int y = 0; y < m.height(); ++y) {
        if (!m.has_wall(0, y, Dir::W))
            return "maze is not enclosed: missing west wall of cell (0," + std::to_string(y) + ")";
        if (!m.has_wall(m.width() - 1, y, Dir::E))
            return "maze is not enclosed: missing east wall of cell (" +
                   std::to_string(m.width() - 1) + "," + std::to_string(y) + ")";
    }
    return std::nullopt;
}

/// Builds a Maze from raw per-cell bytes (row-major, y*W+x), checking shared-wall consistency.
/// `line_of` maps a cell to the source line for error messages (0 if unknown).
template <typename LineOf>
MazeLoadResult from_raw(int w, int h, const std::vector<std::uint8_t>& raw, LineOf line_of) {
    auto at = [&](int x, int y) { return raw[static_cast<std::size_t>(y * w + x)]; };
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            for (Dir d : {Dir::N, Dir::E}) {
                const int nx = x + dx(d), ny = y + dy(d);
                if (nx >= w || ny >= h) continue;
                const bool mine = (at(x, y) & wall_bit(d)) != 0;
                const bool theirs = (at(nx, ny) & wall_bit(opposite(d))) != 0;
                if (mine != theirs)
                    return MazeLoadResult::fail(line_of(x, y), 0,
                                                "inconsistent wall between cell (" +
                                                    std::to_string(x) + "," + std::to_string(y) +
                                                    ") and (" + std::to_string(nx) + "," +
                                                    std::to_string(ny) + ")");
            }
        }
    }
    Maze m(w, h);
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            for (Dir d : kAllDirs)
                if (at(x, y) & wall_bit(d)) m.set_wall(x, y, d, true);
    if (auto e = enclosure_error(m)) return MazeLoadResult::fail(0, 0, *e);
    m.set_start({0, 0}, Dir::N);
    m.infer_start_heading();
    m.set_default_goals();
    return {std::move(m), {}};
}

bool parse_int(std::string_view tok, int& out) {
    auto [p, ec] = std::from_chars(tok.data(), tok.data() + tok.size(), out);
    return ec == std::errc{} && p == tok.data() + tok.size();
}

}  // namespace

MazeLoadResult parse_text_map(std::string_view text) {
    std::vector<std::string_view> lines = split_lines(text);
    while (!lines.empty() && is_blank(lines.back())) lines.pop_back();
    std::size_t first = 0;
    while (first < lines.size() && is_blank(lines[first])) ++first;
    lines.erase(lines.begin(), lines.begin() + static_cast<std::ptrdiff_t>(first));
    if (lines.empty()) return MazeLoadResult::fail(0, 0, "empty maze file");

    const int rows = static_cast<int>(lines.size());
    if (rows < 3 || rows % 2 == 0)
        return MazeLoadResult::fail(
            rows, 0, "row count must be odd (2*height+1), got " + std::to_string(rows));
    std::size_t longest = 0;
    int longest_line = 1;
    for (int i = 0; i < rows; ++i) {
        if (lines[static_cast<std::size_t>(i)].size() > longest) {
            longest = lines[static_cast<std::size_t>(i)].size();
            longest_line = static_cast<int>(first) + i + 1;
        }
    }
    const int cols = static_cast<int>(longest);
    if (cols < 5 || (cols - 1) % 4 != 0)
        return MazeLoadResult::fail(longest_line, cols,
                                    "line width must be 4*width+1, got " + std::to_string(cols));

    const int h = (rows - 1) / 2, w = (cols - 1) / 4;
    if (w > kMaxDim || h > kMaxDim) return MazeLoadResult::fail(0, 0, "maze larger than 255x255");

    std::vector<std::string> grid;
    grid.reserve(lines.size());
    for (auto l : lines) {
        std::string s(l);
        s.resize(static_cast<std::size_t>(cols), ' ');
        grid.push_back(std::move(s));
    }
    auto ch = [&](int r, int c) {
        return grid[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)];
    };

    Maze m(w, h);
    std::optional<Cell> start;
    std::vector<Cell> goals;
    for (int y = 0; y < h; ++y) {
        const int r = 2 * (h - 1 - y) + 1;  // text row of the cell centre
        for (int x = 0; x < w; ++x) {
            const int c = 4 * x + 2;  // text column of the cell centre
            if (ch(r - 1, c) != ' ') m.set_wall(x, y, Dir::N, true);
            if (ch(r + 1, c) != ' ') m.set_wall(x, y, Dir::S, true);
            if (ch(r, c + 2) != ' ') m.set_wall(x, y, Dir::E, true);
            if (ch(r, c - 2) != ' ') m.set_wall(x, y, Dir::W, true);
            for (int k = c - 1; k <= c + 1; ++k) {
                const char v = ch(r, k);
                if (v == 'S') {
                    if (start && !(*start == Cell{x, y}))
                        return MazeLoadResult::fail(static_cast<int>(first) + r + 1, k + 1,
                                                    "more than one start cell 'S'");
                    start = Cell{x, y};
                } else if (v == 'G') {
                    goals.push_back({x, y});
                }
            }
        }
    }
    if (auto e = enclosure_error(m)) return MazeLoadResult::fail(0, 0, *e);

    m.set_start(start.value_or(Cell{0, 0}), Dir::N);
    m.infer_start_heading();
    if (goals.empty())
        m.set_default_goals();
    else
        m.set_goals(std::move(goals));
    return {std::move(m), {}};
}

MazeLoadResult parse_num(std::string_view text) {
    struct Entry {
        int x, y, line;
        std::uint8_t bits;
    };
    std::vector<Entry> entries;
    int lineno = 0;
    for (std::string_view line : split_lines(text)) {
        ++lineno;
        if (is_blank(line) || line.front() == '#') continue;
        std::vector<std::string_view> tok;
        std::size_t p = 0;
        while (p < line.size()) {
            while (p < line.size() && (line[p] == ' ' || line[p] == '\t')) ++p;
            std::size_t q = p;
            while (q < line.size() && line[q] != ' ' && line[q] != '\t') ++q;
            if (q > p) tok.push_back(line.substr(p, q - p));
            p = q;
        }
        if (tok.size() != 6)
            return MazeLoadResult::fail(
                lineno, 0, "expected 'X Y N E S W', got " + std::to_string(tok.size()) + " fields");
        int v[6];
        for (int i = 0; i < 6; ++i) {
            if (!parse_int(tok[static_cast<std::size_t>(i)], v[i]))
                return MazeLoadResult::fail(
                    lineno, 0,
                    "not an integer: '" + std::string(tok[static_cast<std::size_t>(i)]) + "'");
            if (i >= 2 && v[i] != 0 && v[i] != 1)
                return MazeLoadResult::fail(lineno, 0, "wall flags must be 0 or 1");
        }
        if (v[0] < 0 || v[1] < 0 || v[0] >= kMaxDim || v[1] >= kMaxDim)
            return MazeLoadResult::fail(lineno, 0, "cell coordinate out of range");
        std::uint8_t bits = 0;
        if (v[2]) bits |= wall_bit(Dir::N);
        if (v[3]) bits |= wall_bit(Dir::E);
        if (v[4]) bits |= wall_bit(Dir::S);
        if (v[5]) bits |= wall_bit(Dir::W);
        entries.push_back({v[0], v[1], lineno, bits});
    }
    if (entries.empty()) return MazeLoadResult::fail(0, 0, "empty maze file");

    int w = 0, h = 0;
    for (const auto& e : entries) {
        w = std::max(w, e.x + 1);
        h = std::max(h, e.y + 1);
    }
    std::vector<std::uint8_t> raw(static_cast<std::size_t>(w * h), 0);
    std::vector<int> line_of(static_cast<std::size_t>(w * h), 0);
    for (const auto& e : entries) {
        auto i = static_cast<std::size_t>(e.y * w + e.x);
        if (line_of[i] != 0)
            return MazeLoadResult::fail(
                e.line, 0,
                "duplicate cell (" + std::to_string(e.x) + "," + std::to_string(e.y) + ")");
        raw[i] = e.bits;
        line_of[i] = e.line;
    }
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            if (line_of[static_cast<std::size_t>(y * w + x)] == 0)
                return MazeLoadResult::fail(0, 0,
                                            "missing cell (" + std::to_string(x) + "," +
                                                std::to_string(y) + "); maze must be rectangular");
    return from_raw(w, h, raw,
                    [&](int x, int y) { return line_of[static_cast<std::size_t>(y * w + x)]; });
}

MazeLoadResult parse_maz(std::span<const std::uint8_t> bytes) {
    int n = 0;
    while ((n + 1) * (n + 1) <= static_cast<int>(bytes.size())) ++n;
    if (n > kMaxDim)
        return MazeLoadResult::fail(0, 0,
                                    ".maz dimension limit exceeded (" + std::to_string(n) + " x " +
                                        std::to_string(n) + ", max " + std::to_string(kMaxDim) +
                                        ")");
    if (n == 0 || n * n != static_cast<int>(bytes.size()))
        return MazeLoadResult::fail(
            0, 0, ".maz size must be N*N bytes, got " + std::to_string(bytes.size()));
    std::vector<std::uint8_t> raw(bytes.size());
    for (int x = 0; x < n; ++x)
        for (int y = 0; y < n; ++y)
            raw[static_cast<std::size_t>(y * n + x)] =
                static_cast<std::uint8_t>(bytes[static_cast<std::size_t>(x * n + y)] & 0x0F);
    return from_raw(n, n, raw, [](int, int) { return 0; });
}

MazeLoadResult load_maze_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return MazeLoadResult::fail(0, 0, "cannot open " + path.string());
    std::string data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::string ext = path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (ext == ".maz") {
        std::vector<std::uint8_t> bytes(data.begin(), data.end());
        return parse_maz(bytes);
    }
    if (ext == ".num") return parse_num(data);
    return parse_text_map(data);
}

std::string write_text_map(const Maze& m) {
    const int w = m.width(), h = m.height();
    const int rows = 2 * h + 1, cols = 4 * w + 1;
    std::vector<std::string> g(static_cast<std::size_t>(rows),
                               std::string(static_cast<std::size_t>(cols), ' '));
    auto put = [&](int r, int c, char v) {
        g[static_cast<std::size_t>(r)][static_cast<std::size_t>(c)] = v;
    };
    for (int r = 0; r < rows; r += 2)
        for (int c = 0; c < cols; c += 4) put(r, c, 'o');
    for (int y = 0; y < h; ++y) {
        const int r = 2 * (h - 1 - y) + 1;
        for (int x = 0; x < w; ++x) {
            const int c = 4 * x + 2;
            if (m.has_wall(x, y, Dir::N))
                for (int k = -1; k <= 1; ++k) put(r - 1, c + k, '-');
            if (m.has_wall(x, y, Dir::S))
                for (int k = -1; k <= 1; ++k) put(r + 1, c + k, '-');
            if (m.has_wall(x, y, Dir::W)) put(r, c - 2, '|');
            if (m.has_wall(x, y, Dir::E)) put(r, c + 2, '|');
            if (m.is_goal({x, y})) put(r, c, 'G');
            if (m.start() == Cell{x, y}) put(r, c, 'S');
        }
    }
    std::string out;
    for (const auto& line : g) {
        out += line;
        out += '\n';
    }
    return out;
}

std::string write_num(const Maze& m) {
    std::ostringstream os;
    for (int x = 0; x < m.width(); ++x)
        for (int y = 0; y < m.height(); ++y)
            os << x << ' ' << y << ' ' << m.has_wall(x, y, Dir::N) << ' '
               << m.has_wall(x, y, Dir::E) << ' ' << m.has_wall(x, y, Dir::S) << ' '
               << m.has_wall(x, y, Dir::W) << '\n';
    return os.str();
}

std::vector<std::uint8_t> write_maz(const Maze& m) {
    if (m.width() != m.height()) throw std::invalid_argument(".maz requires a square maze");
    const int n = m.width();
    std::vector<std::uint8_t> out(static_cast<std::size_t>(n * n));
    for (int x = 0; x < n; ++x)
        for (int y = 0; y < n; ++y) out[static_cast<std::size_t>(x * n + y)] = m.walls(x, y);
    return out;
}

}  // namespace mmsim::core
