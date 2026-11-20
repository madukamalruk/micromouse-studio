#pragma once
// Design tokens (DESIGN.md, Slice 9.10). The only place colours are defined; everything
// else refers to these names. No ImGui dependency (used by pixel checks and tests too).

#include <cstdint>

namespace mmsim::app {

/// RGBA8 colours, ImGui IM_COL32 byte order (R in the low byte).
struct Rgba {
    std::uint8_t r, g, b, a;
    constexpr std::uint32_t packed() const {
        return std::uint32_t(r) | (std::uint32_t(g) << 8) | (std::uint32_t(b) << 16) |
               (std::uint32_t(a) << 24);
    }
    constexpr Rgba with_alpha(float f) const {
        return {r, g, b, static_cast<std::uint8_t>(f <= 0 ? 0 : f >= 1 ? 255 : f * 255.0f + 0.5f)};
    }
};

/// Linear mix a -> b (t in 0..1), opaque.
constexpr Rgba mix(Rgba a, Rgba b, float t) {
    auto m = [t](std::uint8_t x, std::uint8_t y) {
        return static_cast<std::uint8_t>(x + (static_cast<float>(y) - x) * t + 0.5f);
    };
    return {m(a.r, b.r), m(a.g, b.g), m(a.b, b.b), 255};
}

namespace tok {
inline constexpr Rgba canvas{14, 17, 22, 255};        // #0E1116
inline constexpr Rgba panel{21, 25, 33, 255};         // #151921
inline constexpr Rgba raised{28, 34, 45, 255};        // #1C222D
inline constexpr Rgba pressed{35, 43, 56, 255};       // #232B38
inline constexpr Rgba hairline{38, 45, 58, 255};      // #262D3A
inline constexpr Rgba text{241, 245, 249, 255};       // #F1F5F9
inline constexpr Rgba muted{139, 150, 168, 255};      // #8B96A8
inline constexpr Rgba accent{56, 189, 248, 255};      // #38BDF8
inline constexpr Rgba good{34, 197, 94, 255};         // #22C55E
inline constexpr Rgba warn{245, 158, 11, 255};        // #F59E0B
inline constexpr Rgba bad{239, 68, 68, 255};          // #EF4444
inline constexpr Rgba oled{5, 8, 12, 255};            // faceplate display glass
inline constexpr Rgba oled_text{186, 230, 253, 255};  // #BAE6FD, cyan-white OLED pixels
}  // namespace tok

}  // namespace mmsim::app
