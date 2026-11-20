#include "sim/app/theme.hpp"

#include <imgui.h>
#include <implot.h>

#include <cmath>
#include <system_error>

namespace mmsim::app::theme {

namespace {
Fonts g_fonts;
bool g_motion = true;
}  // namespace

const Fonts& fonts() {
    return g_fonts;
}

ImVec4 v4(Rgba c, float a) {
    return {c.r / 255.0f, c.g / 255.0f, c.b / 255.0f, (c.a / 255.0f) * a};
}

unsigned u32(Rgba c, float a) {
    return c.with_alpha((c.a / 255.0f) * a).packed();
}

float ease(float cur, float target, float dt, bool instant, float tau) {
    if (instant || !g_motion || tau <= 0.0f) return target;
    const float k = 1.0f - std::exp(-3.0f * dt / tau);
    return cur + (target - cur) * k;
}

void set_motion_enabled(bool on) {
    g_motion = on;
}
bool motion_enabled() {
    return g_motion;
}

void apply(float scale, const std::filesystem::path& font_dir) {
    ImGuiIO& io = ImGui::GetIO();
    std::error_code ec;
    const auto ui = font_dir / "Roboto-Medium.ttf";
    const auto mono = font_dir / "Cousine-Regular.ttf";
    if (std::filesystem::is_regular_file(ui, ec) && std::filesystem::is_regular_file(mono, ec)) {
        g_fonts.ui = io.Fonts->AddFontFromFileTTF(ui.string().c_str(), kUiSize);
        g_fonts.mono = io.Fonts->AddFontFromFileTTF(mono.string().c_str(), kUiSize);
        g_fonts.bundled = g_fonts.ui && g_fonts.mono;
    }
    if (!g_fonts.bundled) {
        g_fonts.ui = io.Fonts->AddFontDefault();
        g_fonts.mono = g_fonts.ui;
    }
    io.FontDefault = g_fonts.ui;

    ImGuiStyle& s = ImGui::GetStyle();
    s = ImGuiStyle();
    s.FontSizeBase = kUiSize;
    s.WindowPadding = {12, 12};
    s.FramePadding = {10, 6};
    s.CellPadding = {8, 5};
    s.ItemSpacing = {8, 8};
    s.ItemInnerSpacing = {6, 6};
    s.IndentSpacing = 16;
    s.ScrollbarSize = 12;
    s.GrabMinSize = 12;
    s.WindowBorderSize = 1;
    s.ChildBorderSize = 1;
    s.PopupBorderSize = 1;
    s.FrameBorderSize = 0;
    s.TabBorderSize = 0;
    s.WindowRounding = 8;
    s.ChildRounding = 8;
    s.FrameRounding = 6;
    s.PopupRounding = 8;
    s.ScrollbarRounding = 6;
    s.GrabRounding = 6;
    s.TabRounding = 6;
    s.WindowTitleAlign = {0.0f, 0.5f};
    s.SeparatorTextBorderSize = 1;
    s.SeparatorTextPadding = {0, 6};
    s.DockingSeparatorSize = 2;
    s.WindowMenuButtonPosition = ImGuiDir_None;

    ImVec4* c = s.Colors;
    const ImVec4 clear{0, 0, 0, 0};
    c[ImGuiCol_Text] = v4(tok::text);
    c[ImGuiCol_TextDisabled] = v4(tok::muted);
    c[ImGuiCol_WindowBg] = v4(tok::panel);
    c[ImGuiCol_ChildBg] = clear;
    c[ImGuiCol_PopupBg] = v4(tok::raised);
    c[ImGuiCol_Border] = v4(tok::hairline);
    c[ImGuiCol_BorderShadow] = clear;
    c[ImGuiCol_FrameBg] = v4(tok::raised);
    c[ImGuiCol_FrameBgHovered] = v4(tok::pressed);
    c[ImGuiCol_FrameBgActive] = v4(tok::pressed);
    c[ImGuiCol_TitleBg] = v4(tok::panel);
    c[ImGuiCol_TitleBgActive] = v4(tok::panel);
    c[ImGuiCol_TitleBgCollapsed] = v4(tok::panel);
    c[ImGuiCol_MenuBarBg] = v4(tok::panel);
    c[ImGuiCol_ScrollbarBg] = clear;
    c[ImGuiCol_ScrollbarGrab] = v4(tok::hairline);
    c[ImGuiCol_ScrollbarGrabHovered] = v4(tok::muted, 0.6f);
    c[ImGuiCol_ScrollbarGrabActive] = v4(tok::muted);
    c[ImGuiCol_CheckMark] = v4(tok::accent);
    c[ImGuiCol_SliderGrab] = v4(tok::accent);
    c[ImGuiCol_SliderGrabActive] = v4(tok::text);
    c[ImGuiCol_Button] = v4(tok::raised);
    c[ImGuiCol_ButtonHovered] = v4(tok::pressed);
    c[ImGuiCol_ButtonActive] = v4(tok::hairline);
    c[ImGuiCol_Header] = v4(tok::accent, 0.16f);
    c[ImGuiCol_HeaderHovered] = v4(tok::accent, 0.22f);
    c[ImGuiCol_HeaderActive] = v4(tok::accent, 0.30f);
    c[ImGuiCol_Separator] = v4(tok::hairline);
    c[ImGuiCol_SeparatorHovered] = v4(tok::accent, 0.6f);
    c[ImGuiCol_SeparatorActive] = v4(tok::accent);
    c[ImGuiCol_ResizeGrip] = clear;
    c[ImGuiCol_ResizeGripHovered] = v4(tok::accent, 0.5f);
    c[ImGuiCol_ResizeGripActive] = v4(tok::accent);
    c[ImGuiCol_InputTextCursor] = v4(tok::accent);
    c[ImGuiCol_Tab] = v4(tok::panel);
    c[ImGuiCol_TabHovered] = v4(tok::raised);
    c[ImGuiCol_TabSelected] = v4(tok::panel);
    c[ImGuiCol_TabSelectedOverline] = clear;
    c[ImGuiCol_TabDimmed] = v4(tok::panel);
    c[ImGuiCol_TabDimmedSelected] = v4(tok::panel);
    c[ImGuiCol_TabDimmedSelectedOverline] = clear;
    c[ImGuiCol_DockingPreview] = v4(tok::accent, 0.35f);
    c[ImGuiCol_DockingEmptyBg] = v4(tok::canvas);
    c[ImGuiCol_PlotLines] = v4(tok::accent);
    c[ImGuiCol_PlotHistogram] = v4(tok::accent);
    c[ImGuiCol_TableHeaderBg] = v4(tok::raised);
    c[ImGuiCol_TableBorderStrong] = v4(tok::hairline);
    c[ImGuiCol_TableBorderLight] = v4(tok::hairline, 0.6f);
    c[ImGuiCol_TableRowBg] = clear;
    c[ImGuiCol_TableRowBgAlt] = v4(tok::text, 0.025f);
    c[ImGuiCol_TextSelectedBg] = v4(tok::accent, 0.30f);
    c[ImGuiCol_TextLink] = v4(tok::accent);
    c[ImGuiCol_DragDropTarget] = v4(tok::accent);
    c[ImGuiCol_NavCursor] = v4(tok::accent);
    c[ImGuiCol_ModalWindowDimBg] = v4(tok::canvas, 0.70f);
    s.ScaleAllSizes(scale);
    s.FontScaleDpi = scale;

    ImPlotStyle& p = ImPlot::GetStyle();
    p.PlotPadding = {8, 8};
    p.LegendPadding = {8, 6};
    p.FitPadding = {0.02f, 0.08f};
    ImVec4* pc = p.Colors;
    pc[ImPlotCol_FrameBg] = clear;
    pc[ImPlotCol_PlotBg] = v4(tok::canvas);
    pc[ImPlotCol_PlotBorder] = v4(tok::hairline);
    pc[ImPlotCol_LegendBg] = v4(tok::raised, 0.92f);
    pc[ImPlotCol_LegendBorder] = v4(tok::hairline);
    pc[ImPlotCol_LegendText] = v4(tok::text);
    pc[ImPlotCol_TitleText] = v4(tok::text);
    pc[ImPlotCol_InlayText] = v4(tok::text);
    pc[ImPlotCol_AxisText] = v4(tok::muted);
    pc[ImPlotCol_AxisGrid] = v4(tok::hairline);
    pc[ImPlotCol_AxisTick] = v4(tok::hairline);
    pc[ImPlotCol_Crosshairs] = v4(tok::muted);
    pc[ImPlotCol_Selection] = v4(tok::accent);
    // Series order: accent, warn, good, pink, violet, text, bad (qualitative, dark-bg safe).
    static const ImVec4 series[] = {
        v4(tok::accent),          v4(tok::warn),        v4(tok::good), v4({244, 114, 182, 255}),
        v4({167, 139, 250, 255}), v4(tok::text, 0.85f), v4(tok::bad)};
    const ImPlotColormap cm = ImPlot::AddColormap("mmsim", series, 7, true);
    p.Colormap = cm;
}

}  // namespace mmsim::app::theme
