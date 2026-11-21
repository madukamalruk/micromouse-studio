#pragma once
// Small ImGui building blocks used by every panel (DESIGN.md): pills, tactile buttons, value
// rows with tabular numerals, bars, LEDs, and a flow layout that wraps instead of clipping.

#include <string>

#include "sim/app/help_text.hpp"
#include "sim/app/palette.hpp"
#include "sim/app/ui_logic.hpp"

struct ImVec2;

namespace mmsim::app::ui {

Rgba tone_color(Tone t);

/// Width of a framed button showing `label` (text + frame padding).
float button_width(const char* label);
/// Wrapping layout: call before an item of width w (not the first in a row): stays on the
/// line if it fits in the window, else starts a new line. Never clips.
void flow(float w);

/// Rounded pill (status / chip). Returns the pill width.
float pill(const char* text, Tone tone, bool strong = true);
/// Selectable pill (speed / workspace segments). Returns true when clicked.
bool pill_toggle(const char* label, bool selected, const char* tooltip = nullptr);
/// Solid action button (GO / STOP / RESET). `tone` colours the fill; disabled dims it.
bool action_button(const char* label, Tone tone, bool disabled = false, float min_width = 0.0f);

/// Muted label + right-aligned mono value (tabular numerals, never jitters).
void value_row(const char* label, const char* fmt, ...);
/// Section heading with a hairline (SeparatorText in the UI font, muted).
void section(const char* title);
/// Same, with a (?) tooltip right after the title.
void section(const char* title, Help h);
/// "(?)" after the previous item on the same line; hover shows the plain-English tooltip.
/// frame_aligned: the previous item is a framed widget (checkbox, drag field): centre vertically.
void help(Help h, bool frame_aligned = false);
/// A (?) hotspot drawn at a screen position with no layout item (inside plots/subplots).
void help_hotspot(float screen_x, float screen_y, Help h);
/// Muted label with a (?) + right-aligned mono value.
void value_row(Help h, const char* label, const char* fmt, ...);
/// `s` shortened with "..." so it fits max_w pixels in the current font.
std::string ellipsize(const std::string& s, float max_w);
/// Wrapped muted help text.
void hint(const char* text);

/// Horizontal bar: frac 0..1 (eased per id), centred text drawn in mono. signed_bar: frac in
/// -1..1 grows from the centre (PWM duty).
void bar(const char* id, float frac, const char* text, Rgba fill, float height = 0.0f,
         bool signed_bar = false, float width = 0.0f);
/// Label (muted, fixed column) + bar on one line, vertically centred on the frame height.
void labeled_bar(const char* label, float label_w, const char* id, float frac, const char* text,
                 Rgba fill, bool signed_bar = false);
/// Width pill() will take for `text`.
float pill_width(const char* text);
/// Circular LED with a soft glow when lit.
void led(const char* id, bool on, Rgba colour, float radius);

/// Monospace text helpers.
void mono(const char* fmt, ...);
void mono_colored(Rgba c, const char* fmt, ...);

/// Simple per-id smoothed value storage for gauges (ease-out 160 ms, see theme::ease).
float eased(const char* id, float target);

}  // namespace mmsim::app::ui
