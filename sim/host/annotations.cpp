#include "sim/host/annotations.hpp"

namespace mmsim::host {

void Annotations::set_color(int x, int y, char c) {
    if (!inside(x, y) || c == '\0' || kPalette.find(c) == std::string_view::npos) return;
    color[{x, y}] = c;
}

void Annotations::set_text(int x, int y, std::string_view t) {
    if (!inside(x, y)) return;
    if (t.empty()) {
        text.erase({x, y});
        return;
    }
    text[{x, y}] = std::string(t.substr(0, kMaxText));
}

std::tuple<int, int, int> Annotations::key(int x, int y, core::Dir d) {
    switch (d) {
        case core::Dir::N:
            return {x, y, 0};
        case core::Dir::E:
            return {x, y, 1};
        case core::Dir::S:
            return {x, y - 1, 0};
        case core::Dir::W:
            return {x - 1, y, 1};
    }
    return {x, y, 0};
}

void Annotations::set_wall(int x, int y, char dir, bool present) {
    if (!inside(x, y)) return;
    core::Dir d;
    switch (dir) {
        case 'n':
        case 'N':
            d = core::Dir::N;
            break;
        case 'e':
        case 'E':
            d = core::Dir::E;
            break;
        case 's':
        case 'S':
            d = core::Dir::S;
            break;
        case 'w':
        case 'W':
            d = core::Dir::W;
            break;
        default:
            return;
    }
    walls[key(x, y, d)] = present;
}

void Annotations::clear() {
    color.clear();
    text.clear();
    walls.clear();
}

std::optional<bool> Annotations::wall(int x, int y, core::Dir d) const {
    const auto it = walls.find(key(x, y, d));
    if (it == walls.end()) return std::nullopt;
    return it->second;
}

bool mms_color_rgb(char c, std::uint32_t& rgb) {
    // mms palette: k black, b blue, a gray, c cyan, g green, o orange, r red, w white,
    // y yellow; upper case = dark variant.
    switch (c) {
        case 'k':
            rgb = 0x000000;
            return true;
        case 'b':
            rgb = 0x0000FF;
            return true;
        case 'a':
            rgb = 0x808080;
            return true;
        case 'c':
            rgb = 0x00FFFF;
            return true;
        case 'g':
            rgb = 0x00FF00;
            return true;
        case 'o':
            rgb = 0xFFA500;
            return true;
        case 'r':
            rgb = 0xFF0000;
            return true;
        case 'w':
            rgb = 0xFFFFFF;
            return true;
        case 'y':
            rgb = 0xFFFF00;
            return true;
        case 'B':
            rgb = 0x00008B;
            return true;
        case 'A':
            rgb = 0x404040;
            return true;
        case 'C':
            rgb = 0x008B8B;
            return true;
        case 'G':
            rgb = 0x006400;
            return true;
        case 'O':
            rgb = 0xFF8C00;
            return true;
        case 'R':
            rgb = 0x8B0000;
            return true;
        case 'W':
            rgb = 0xC0C0C0;
            return true;
        case 'Y':
            rgb = 0x9B870C;
            return true;
        default:
            return false;
    }
}

}  // namespace mmsim::host
