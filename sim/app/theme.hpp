#pragma once
// ImGui / ImPlot styling from the design tokens (DESIGN.md) and the bundled fonts.

#include <filesystem>

#include "sim/app/palette.hpp"

struct ImFont;
struct ImVec4;

namespace mmsim::app::theme {

struct Fonts {
    ImFont* ui = nullptr;    // Roboto, UI text (tabular digits)
    ImFont* mono = nullptr;  // Cousine, live numbers / log / console
    bool bundled = false;    // false: ImGui's default font (fonts/ folder missing)
};
const Fonts& fonts();

constexpr float kUiSize = 15.0f;
constexpr float kSmallSize = 13.0f;
constexpr float kTitleSize = 18.0f;
constexpr float kOledSize = 16.0f;

/// Applies the style (spacing 12/8, rounding 6/8, colours) and loads fonts from font_dir
/// (Roboto-Medium.ttf, Cousine-Regular.ttf). Call once after ImGui/ImPlot contexts exist.
void apply(float dpi_scale, const std::filesystem::path& font_dir);

ImVec4 v4(Rgba c, float alpha = 1.0f);
unsigned u32(Rgba c, float alpha = 1.0f);

/// Exponential ease-out towards target: ~95 % of the way after `tau` seconds (160 ms default).
/// instant = true (keyboard actions, scripted captures) jumps straight to the target.
float ease(float current, float target, float dt, bool instant, float tau = 0.16f);

/// Global switch: scripted/screenshot runs draw every gauge at its final value.
void set_motion_enabled(bool on);
bool motion_enabled();

}  // namespace mmsim::app::theme
