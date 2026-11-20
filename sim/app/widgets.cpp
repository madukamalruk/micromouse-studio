#include "sim/app/widgets.hpp"

#include <imgui.h>
#include <imgui_internal.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <unordered_map>

#include "sim/app/theme.hpp"

namespace mmsim::app::ui {

namespace {
std::string vfmt(const char* fmt, va_list ap) {
    char buf[512];
    std::vsnprintf(buf, sizeof buf, fmt, ap);
    return buf;
}
}  // namespace

Rgba tone_color(Tone t) {
    switch (t) {
        case Tone::Active:
            return tok::accent;
        case Tone::Good:
            return tok::good;
        case Tone::Warn:
            return tok::warn;
        case Tone::Bad:
            return tok::bad;
        case Tone::Neutral:
            break;
    }
    return tok::muted;
}

float button_width(const char* label) {
    return ImGui::CalcTextSize(label, nullptr, true).x + ImGui::GetStyle().FramePadding.x * 2.0f;
}

void flow(float w) {
    const ImGuiStyle& s = ImGui::GetStyle();
    const float right_edge = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
    const float next_x = ImGui::GetItemRectMax().x + s.ItemSpacing.x;
    if (next_x + w <= right_edge + 0.5f) ImGui::SameLine();
}

float pill(const char* text, Tone tone, bool strong) {
    const Rgba c = tone_color(tone);
    const ImVec2 ts = ImGui::CalcTextSize(text);
    const ImGuiStyle& s = ImGui::GetStyle();
    const ImVec2 pad{s.FramePadding.x + 2.0f, s.FramePadding.y};
    const ImVec2 size{ts.x + pad.x * 2.0f + 14.0f, ts.y + pad.y * 2.0f};
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p, {p.x + size.x, p.y + size.y}, theme::u32(c, strong ? 0.16f : 0.08f),
                      size.y * 0.5f);
    dl->AddRect(p, {p.x + size.x, p.y + size.y}, theme::u32(c, 0.45f), size.y * 0.5f, 0, 1.0f);
    const float r = 3.5f;
    dl->AddCircleFilled({p.x + pad.x + r, p.y + size.y * 0.5f}, r, theme::u32(c));
    dl->AddText({p.x + pad.x + 14.0f, p.y + pad.y},
                theme::u32(tone == Tone::Neutral ? tok::text : c), text);
    ImGui::Dummy(size);
    return size.x;
}

bool pill_toggle(const char* label, bool selected, const char* tooltip) {
    ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 999.0f);
    ImGui::PushStyleColor(ImGuiCol_Button,
                          theme::v4(selected ? tok::accent : tok::raised, selected ? 0.22f : 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::v4(selected ? tok::accent : tok::pressed,
                                                            selected ? 0.30f : 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive, theme::v4(tok::accent, 0.38f));
    ImGui::PushStyleColor(ImGuiCol_Text, theme::v4(selected ? tok::accent : tok::text));
    const bool hit = ImGui::Button(label);
    ImGui::PopStyleColor(4);
    ImGui::PopStyleVar();
    if (tooltip && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal))
        ImGui::SetTooltip("%s", tooltip);
    return hit;
}

bool action_button(const char* label, Tone tone, bool disabled, float min_width) {
    const Rgba c = tone_color(tone);
    const bool solid = tone == Tone::Good || tone == Tone::Bad;
    ImGui::BeginDisabled(disabled);
    ImGui::PushStyleColor(ImGuiCol_Button,
                          theme::v4(solid ? c : tok::raised, solid ? 0.90f : 1.0f));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, theme::v4(solid ? c : tok::pressed));
    ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                          theme::v4(solid ? mix(c, tok::canvas, 0.25f) : tok::hairline));
    ImGui::PushStyleColor(ImGuiCol_Text, theme::v4(solid ? tok::canvas : tok::text));
    const float w = std::max(min_width, button_width(label));
    const bool hit = ImGui::Button(label, {w, 0});
    ImGui::PopStyleColor(4);
    ImGui::EndDisabled();
    return hit && !disabled;
}

void value_row(const char* label, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    const std::string v = vfmt(fmt, ap);
    va_end(ap);
    ImGui::TextDisabled("%s", label);
    ImGui::PushFont(theme::fonts().mono, 0.0f);
    const float w = ImGui::CalcTextSize(v.c_str()).x;
    const float right = ImGui::GetWindowContentRegionMax().x;
    ImGui::SameLine(std::max(ImGui::GetCursorPosX() + 8.0f, right - w));
    ImGui::TextUnformatted(v.c_str());
    ImGui::PopFont();
}

void section(const char* title) {
    ImGui::Dummy({0, 2});
    ImGui::PushStyleColor(ImGuiCol_Text, theme::v4(tok::muted));
    ImGui::SeparatorText(title);
    ImGui::PopStyleColor();
}

void hint(const char* text) {
    ImGui::PushStyleColor(ImGuiCol_Text, theme::v4(tok::muted));
    ImGui::PushTextWrapPos(0.0f);
    ImGui::TextUnformatted(text);
    ImGui::PopTextWrapPos();
    ImGui::PopStyleColor();
}

float eased(const char* id, float target) {
    static std::unordered_map<ImGuiID, float> values;
    const ImGuiID key = ImGui::GetID(id);
    auto it = values.find(key);
    if (it == values.end() || !std::isfinite(it->second)) {
        values[key] = target;
        return target;
    }
    it->second = theme::ease(it->second, target, ImGui::GetIO().DeltaTime, false);
    return it->second;
}

void bar(const char* id, float frac, const char* text, Rgba fill, float height, bool signed_bar,
         float width) {
    const float h = height > 0 ? height : ImGui::GetFrameHeight() * 0.8f;
    const float w = width > 0 ? width : ImGui::GetContentRegionAvail().x;
    const ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const float f =
        eased(id, std::isfinite(frac) ? std::clamp(frac, signed_bar ? -1.0f : 0.0f, 1.0f) : 0.0f);
    dl->AddRectFilled(p, {p.x + w, p.y + h}, theme::u32(tok::raised), 4.0f);
    if (signed_bar) {
        const float mid = p.x + w * 0.5f;
        const float x = mid + f * w * 0.5f;
        dl->AddRectFilled({std::min(mid, x), p.y}, {std::max(mid, x), p.y + h},
                          theme::u32(fill, 0.85f), 3.0f);
        dl->AddLine({mid, p.y}, {mid, p.y + h}, theme::u32(tok::muted, 0.6f));
    } else if (f > 0.0f) {
        dl->AddRectFilled(p, {p.x + std::max(4.0f, w * f), p.y + h}, theme::u32(fill, 0.85f), 4.0f);
    }
    if (text && *text) {
        ImGui::PushFont(theme::fonts().mono, theme::kSmallSize);
        const ImVec2 ts = ImGui::CalcTextSize(text);
        const ImVec2 tp{p.x + w - ts.x - 6.0f, p.y + (h - ts.y) * 0.5f};
        // Backing chip keeps the value readable where it overlaps the fill.
        dl->AddRectFilled({tp.x - 4.0f, p.y + 1.0f}, {p.x + w - 1.0f, p.y + h - 1.0f},
                          theme::u32(tok::raised, 0.85f), 3.0f);
        dl->AddText(tp, theme::u32(tok::text), text);
        ImGui::PopFont();
    }
    ImGui::Dummy({w, h});
}

void labeled_bar(const char* label, float label_w, const char* id, float frac, const char* text,
                 Rgba fill, bool signed_bar) {
    const float fh = ImGui::GetFrameHeight();
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("%s", label);
    ImGui::SameLine(label_w);
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + fh * 0.1f);
    bar(id, frac, text, fill, fh * 0.8f, signed_bar);
}

float pill_width(const char* text) {
    const ImGuiStyle& s = ImGui::GetStyle();
    return ImGui::CalcTextSize(text).x + (s.FramePadding.x + 2.0f) * 2.0f + 14.0f;
}

void led(const char* id, bool on, Rgba colour, float radius) {
    const ImVec2 p = ImGui::GetCursorScreenPos();
    const float glow = eased(id, on ? 1.0f : 0.0f);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImVec2 c{p.x + radius * 1.8f, p.y + radius * 1.8f};
    if (glow > 0.01f) {
        dl->AddCircleFilled(c, radius * 1.8f, theme::u32(colour, 0.10f * glow));
        dl->AddCircleFilled(c, radius * 1.35f, theme::u32(colour, 0.22f * glow));
    }
    dl->AddCircleFilled(c, radius, theme::u32(mix(tok::raised, colour, glow)));
    dl->AddCircle(c, radius, theme::u32(on ? colour : tok::hairline), 0, 1.0f);
    ImGui::Dummy({radius * 3.6f, radius * 3.6f});
}

void mono(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    const std::string v = vfmt(fmt, ap);
    va_end(ap);
    ImGui::PushFont(theme::fonts().mono, 0.0f);
    ImGui::TextUnformatted(v.c_str());
    ImGui::PopFont();
}

void mono_colored(Rgba c, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    const std::string v = vfmt(fmt, ap);
    va_end(ap);
    ImGui::PushFont(theme::fonts().mono, 0.0f);
    ImGui::TextColored(theme::v4(c), "%s", v.c_str());
    ImGui::PopFont();
}

}  // namespace mmsim::app::ui
