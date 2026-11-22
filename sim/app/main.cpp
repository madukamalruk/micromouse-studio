// MMSim GUI: real-time SIL micromouse simulator. Slice 9.10 UI: slim top bar + four
// workspaces (Cockpit, Maze Studio, Telemetry & Compare, Tuning & Reports), one action table
// behind shortcuts, console and help (DESIGN.md).
#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <imgui_internal.h>  // DockBuilder, BeginViewportSideBar
#include <implot.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "sim/app/app_state.hpp"
#include "sim/app/audio.hpp"
#include "sim/app/panels.hpp"
#include "sim/app/screenshot.hpp"
#include "sim/app/settings.hpp"
#include "sim/app/theme.hpp"
#include "sim/app/user_paths.hpp"
#include "sim/app/widgets.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/core/version.hpp"
#include "sim/platform/window_chrome.hpp"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include <stb_image.h>

namespace fs = std::filesystem;
using namespace mmsim;
using namespace mmsim::app;

namespace {

// ---------------------------------------------------------------------------------------------
struct Args {
    std::optional<fs::path> maze;
    std::optional<fs::path> screenshot;
    std::optional<fs::path> root;
    std::optional<std::string> firmware;  // NAME (from the firmware dir) or path to a .dll
    std::optional<fs::path> config;
    std::optional<fs::path> export_csv;  // telemetry CSV after --run-for
    std::optional<fs::path> report;      // HTML report after --run-for
    std::optional<fs::path> mms_solver;  // mms-protocol solver exe (stdin/stdout)
    std::optional<std::string> workspace;
    std::optional<std::string> mode;
    bool mms_sensor_walls = false;
    bool save_run = false;
    long long run_for_ms = -1;
    std::uint64_t seed = 1;
    double speed = 1.0;
    double live_for_s = -1.0;
    int frames = -1;
    bool exit_after = false;
    bool hidden = false;
    bool software = false;
    bool verify_render = false;
    bool iso = false;
    bool no_session = false;
    bool portable = false;
    bool seed_given = false, speed_given = false;
    int width = 1280, height = 800;
};

void print_usage() {
    std::printf(
        "mmsim %s - real-time SIL micromouse simulator\n"
        "  --maze FILE          load a maze (.txt map, .num, .maz)\n"
        "  --root DIR           data root containing mazes/ (default: auto-detect)\n"
        "  --firmware NAME|DLL  firmware to run (NAME = build/<preset>/firmware_bin/NAME.dll)\n"
        "  --mms-solver EXE     run an mms-protocol solver (stdin/stdout) instead of firmware\n"
        "  --mms-walls truth|sensors  mms wall queries from the maze (default) or the sensors\n"
        "  --config FILE        robot config JSON (default config/robots/generic_classic.json)\n"
        "  --run-for T          simulate T (e.g. 3s, 250ms) as fast as possible, then render\n"
        "  --seed N             noise seed (default 1): same seed + firmware = same run\n"
        "  --export-csv FILE    after --run-for: write all recorded telemetry as CSV\n"
        "  --report FILE        after --run-for: write the offline HTML engineering report\n"
        "  --save-run           after --run-for: archive the run under <root>/runs/\n"
        "  --workspace NAME     cockpit | studio | telemetry | tuning (keys 1-4)\n"
        "  --mode M             beginner | engineer\n"
        "  --speed S            pacing: 0.1 .. 64 or max (default 1)\n"
        "  --live-for T         play paced for T wall seconds, print sim/wall time, exit\n"
        "  --screenshot PNG     save a screenshot of the last frame\n"
        "  --frames N           render N frames\n"
        "  --exit               exit after --frames\n"
        "  --size WxH           window size (default 1280x800)\n"
        "  --hidden             hidden window (with --software for headless checks)\n"
        "  --iso                start in 2.5D isometric view\n"
        "  --software           SDL software renderer (diagnostic only; drops thin walls)\n"
        "  --verify-render      pixel-check walls/posts against the maze; exit 1 on mismatch\n"
        "  --no-session         do not restore/save the last session (mmsim.json, imgui.ini)\n"
        "  --portable           keep mmsim.json, runs/, reports/ next to the program\n",
        std::string(core::version()).c_str());
}

std::optional<Args> parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string s = argv[i];
        auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
        auto path_opt = [&](std::optional<fs::path>& dst) {
            const char* v = next();
            if (v) dst = v;
            return v != nullptr;
        };
        if (s == "--maze") {
            if (!path_opt(a.maze)) return std::nullopt;
        } else if (s == "--screenshot") {
            if (!path_opt(a.screenshot)) return std::nullopt;
        } else if (s == "--root") {
            if (!path_opt(a.root)) return std::nullopt;
        } else if (s == "--firmware") {
            const char* v = next();
            if (!v) return std::nullopt;
            a.firmware = v;
        } else if (s == "--mms-solver") {
            if (!path_opt(a.mms_solver)) return std::nullopt;
        } else if (s == "--mms-walls") {
            const char* v = next();
            if (!v) return std::nullopt;
            const std::string w = v;
            if (w != "truth" && w != "sensors") return std::nullopt;
            a.mms_sensor_walls = w == "sensors";
        } else if (s == "--export-csv") {
            if (!path_opt(a.export_csv)) return std::nullopt;
        } else if (s == "--report") {
            if (!path_opt(a.report)) return std::nullopt;
        } else if (s == "--save-run") {
            a.save_run = true;
        } else if (s == "--workspace") {
            const char* v = next();
            if (!v || !ui::parse_workspace(v)) return std::nullopt;
            a.workspace = v;
        } else if (s == "--mode") {
            const char* v = next();
            if (!v || !ui::parse_mode(v)) return std::nullopt;
            a.mode = v;
        } else if (s == "--config") {
            if (!path_opt(a.config)) return std::nullopt;
        } else if (s == "--run-for") {
            const char* v = next();
            if (!v) return std::nullopt;
            char* end = nullptr;
            const double t = std::strtod(v, &end);
            const std::string unit = end ? end : "";
            if (end == v || t < 0 || (!unit.empty() && unit != "s" && unit != "ms"))
                return std::nullopt;
            a.run_for_ms = std::llround(unit == "ms" ? t : t * 1000.0);
        } else if (s == "--seed") {
            const char* v = next();
            if (!v) return std::nullopt;
            char* end = nullptr;
            const unsigned long long n = std::strtoull(v, &end, 0);
            if (end == v || *end != '\0') return std::nullopt;
            a.seed = n;
            a.seed_given = true;
        } else if (s == "--speed") {
            const char* v = next();
            if (!v) return std::nullopt;
            if (std::string(v) == "max") {
                a.speed = 0.0;
            } else {
                char* end = nullptr;
                a.speed = std::strtod(v, &end);
                if (end == v || a.speed <= 0.0) return std::nullopt;
            }
        } else if (s == "--no-session") {
            a.no_session = true;
        } else if (s == "--portable") {
            a.portable = true;
        } else if (s == "--live-for") {
            const char* v = next();
            if (!v) return std::nullopt;
            char* end = nullptr;
            a.live_for_s = std::strtod(v, &end);
            if (end == v || a.live_for_s <= 0.0) return std::nullopt;
        } else if (s == "--frames") {
            const char* v = next();
            if (!v) return std::nullopt;
            a.frames = std::max(1, std::atoi(v));
        } else if (s == "--size") {
            const char* v = next();
            if (!v || std::sscanf(v, "%dx%d", &a.width, &a.height) != 2) return std::nullopt;
        } else if (s == "--exit") {
            a.exit_after = true;
        } else if (s == "--hidden") {
            a.hidden = true;
        } else if (s == "--iso") {
            a.iso = true;
        } else if (s == "--software") {
            a.software = true;
        } else if (s == "--verify-render") {
            a.verify_render = true;
        } else {
            return std::nullopt;
        }
    }
    if (a.exit_after && a.frames < 0) a.frames = 5;
    a.speed_given = a.speed != 1.0;
    return a;
}

#include "sim/app/icon_png.inc"

/// The 64 px window icon (also embedded as resource 1 in mmsim.exe for Explorer and the taskbar).
void set_window_icon(SDL_Window* window) {
    int w = 0, h = 0, n = 0;
    unsigned char* px =
        stbi_load_from_memory(kIconPng64, static_cast<int>(sizeof kIconPng64), &w, &h, &n, 4);
    if (!px) return;
    if (SDL_Surface* surf =
            SDL_CreateRGBSurfaceWithFormatFrom(px, w, h, 32, w * 4, SDL_PIXELFORMAT_RGBA32)) {
        SDL_SetWindowIcon(window, surf);
        SDL_FreeSurface(surf);
    }
    stbi_image_free(px);
}

fs::path exe_dir() {
    fs::path p;
    if (char* base = SDL_GetBasePath()) {
        p = base;
        SDL_free(base);
    }
    return p;
}

// --- layouts ----------------------------------------------------------------------------------
const char* dockspace_name(ui::Workspace w) {
    switch (w) {
        case ui::Workspace::Cockpit:
            return "WS_Cockpit";
        case ui::Workspace::Studio:
            return "WS_Studio";
        case ui::Workspace::Telemetry:
            return "WS_Telemetry";
        case ui::Workspace::Tuning:
            return "WS_Tuning";
    }
    return "WS_Cockpit";
}

void build_layout(ui::Workspace w, ui::Mode mode, ImGuiID id, const ImGuiViewport* vp) {
    ImGui::DockBuilderRemoveNode(id);
    ImGui::DockBuilderAddNode(id, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodePos(id, vp->WorkPos);
    ImGui::DockBuilderSetNodeSize(id, vp->WorkSize);
    ImGuiID c = id;
    auto split = [](ImGuiID& node, ImGuiDir dir, float ratio) {
        return ImGui::DockBuilderSplitNode(node, dir, ratio, nullptr, &node);
    };
    switch (w) {
        case ui::Workspace::Cockpit: {
            if (mode == ui::Mode::Beginner) {
                // Plain and roomy: Robot on the left, the maze in the middle, Sensors + Events
                // right.
                ImGuiID left = split(c, ImGuiDir_Left, 0.25f);
                ImGuiID right = split(c, ImGuiDir_Right, 0.34f);
                ImGuiID events = split(right, ImGuiDir_Down, 0.45f);
                ImGui::DockBuilderDockWindow(win::kMazeCockpit, c);
                ImGui::DockBuilderDockWindow(win::kFaceplate, left);
                ImGui::DockBuilderDockWindow(win::kSensors, right);
                ImGui::DockBuilderDockWindow(win::kEvents, events);
            } else {
                // Engineer: Robot + Attitude compass left, maze centre, Sensors above
                // Events|Details.
                ImGuiID left = split(c, ImGuiDir_Left, 0.23f);
                ImGuiID right = split(c, ImGuiDir_Right, 0.36f);
                ImGuiID attitude = split(left, ImGuiDir_Down, 0.28f);
                ImGuiID log = split(right, ImGuiDir_Down, 0.50f);
                ImGui::DockBuilderDockWindow(win::kMazeCockpit, c);
                ImGui::DockBuilderDockWindow(win::kFaceplate, left);
                ImGui::DockBuilderDockWindow(win::kAttitude, attitude);
                ImGui::DockBuilderDockWindow(win::kSensors, right);
                ImGui::DockBuilderDockWindow(win::kEvents, log);
                ImGui::DockBuilderDockWindow(win::kDetails, log);
            }
            break;
        }
        case ui::Workspace::Studio: {
            // Maze | overlays (full height) | deck above the cell inspector.
            ImGuiID right = split(c, ImGuiDir_Right, 0.54f);
            ImGuiID overlays = split(right, ImGuiDir_Left, 0.46f);
            ImGuiID insp = split(right, ImGuiDir_Down, 0.42f);
            ImGui::DockBuilderDockWindow(win::kMazeStudio, c);
            ImGui::DockBuilderDockWindow(win::kOverlays, overlays);
            ImGui::DockBuilderDockWindow(win::kTrajectory, right);
            ImGui::DockBuilderDockWindow(win::kInspector, insp);
            break;
        }
        case ui::Workspace::Telemetry: {
            ImGuiID right = split(c, ImGuiDir_Right, 0.42f);
            ImGuiID traction = split(right, ImGuiDir_Down, 0.45f);
            ImGuiID compare = split(c, ImGuiDir_Down, 0.45f);
            ImGui::DockBuilderDockWindow(win::kTelemetry, c);
            ImGui::DockBuilderDockWindow(win::kCompare, compare);
            ImGui::DockBuilderDockWindow(win::kMazeGhost, right);
            ImGui::DockBuilderDockWindow(win::kTraction, traction);
            break;
        }
        case ui::Workspace::Tuning: {
            // Four columns: mounts over IMU | battery | robot config | timer over
            // reports / step response / run record tabs.
            ImGuiID a = split(c, ImGuiDir_Left, 0.25f);
            ImGuiID b = split(c, ImGuiDir_Left, 0.34f);
            ImGuiID d = split(c, ImGuiDir_Right, 0.50f);
            ImGuiID imu = split(a, ImGuiDir_Down, 0.45f);
            ImGuiID tabs = split(d, ImGuiDir_Down, 0.48f);
            ImGui::DockBuilderDockWindow(win::kMounts, a);
            ImGui::DockBuilderDockWindow(win::kBattery, b);
            ImGui::DockBuilderDockWindow(win::kImu, imu);
            ImGui::DockBuilderDockWindow(win::kConfig, c);
            ImGui::DockBuilderDockWindow(win::kTimer, d);
            ImGui::DockBuilderDockWindow(win::kReports, tabs);
            ImGui::DockBuilderDockWindow(win::kStep, tabs);
            ImGui::DockBuilderDockWindow(win::kRecord, tabs);
            break;
        }
    }
    ImGui::DockBuilderFinish(id);
}

void draw_workspace(AppState& st) {
    switch (st.workspace) {
        case ui::Workspace::Cockpit:
            panel_maze(st, win::kMazeCockpit, CanvasKind::Cockpit);
            panel_faceplate(st);
            panel_sensors(st);
            panel_events(st);
            if (st.mode == ui::Mode::Engineer) {
                panel_attitude(st);
                if (st.details_open) panel_details(st);
            }
            break;
        case ui::Workspace::Studio:
            panel_maze(st, win::kMazeStudio, CanvasKind::Studio);
            panel_overlays(st);
            panel_trajectory(st);
            panel_inspector(st);
            break;
        case ui::Workspace::Telemetry:
            panel_telemetry(st);
            panel_compare(st);
            panel_maze(st, win::kMazeGhost, CanvasKind::Ghost);
            panel_traction(st);
            break;
        case ui::Workspace::Tuning:
            panel_mounts(st);
            panel_imu(st);
            panel_battery(st);
            panel_step(st);
            panel_reports(st);
            panel_record(st);
            panel_config(st);
            panel_timer(st);
            break;
    }
}

// --- top bar ----------------------------------------------------------------------------------
std::string ellipsize(const std::string& s, float max_w) {
    if (ImGui::CalcTextSize(s.c_str()).x <= max_w) return s;
    std::string t = s;
    while (!t.empty() && ImGui::CalcTextSize((t + "...").c_str()).x > max_w) t.pop_back();
    return t + "...";
}

/// Combo whose width fits its preview (never clips): returns BeginCombo's result.
bool fitted_combo(const char* id, const std::string& preview, float max_w) {
    const float arrow = ImGui::GetFrameHeight();
    const float pad = ImGui::GetStyle().FramePadding.x * 2.0f;
    const std::string shown = ellipsize(preview, max_w - arrow - pad);
    ImGui::SetNextItemWidth(ImGui::CalcTextSize(shown.c_str()).x + arrow + pad + 2.0f);
    return ImGui::BeginCombo(id, shown.c_str(), ImGuiComboFlags_HeightLarge);
}

void maze_menu(AppState& st) {
    const std::string name = st.maze ? st.maze_path.stem().string() : std::string("Pick a maze");
    if (fitted_combo("##maze", "Maze  " + name, 220.0f)) {
        ImGui::SetNextItemWidth(260.0f);
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        ImGui::InputTextWithHint("##filter", "filter, e.g. classic, 9x9, speed", st.filter,
                                 sizeof st.filter);
        auto lower = [](std::string s) {
            for (char& ch : s) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            return s;
        };
        const std::string f = lower(st.filter);
        for (const auto& g : st.library) {
            if (g.files.empty()) continue;
            const std::string hdr = g.label + " (" + std::to_string(g.files.size()) + ")";
            ImGui::SeparatorText(hdr.c_str());
            int shown = 0;
            for (const auto& p : g.files) {
                const std::string n = p.filename().string();
                if (!f.empty() && lower(n).find(f) == std::string::npos) continue;
                if (f.empty() && ++shown > 40) {
                    ImGui::TextDisabled("... type to filter %zu mazes", g.files.size());
                    break;
                }
                if (ImGui::Selectable(n.c_str(), p == st.maze_path)) {
                    load_maze(st, p);
                    ImGui::CloseCurrentPopup();
                }
            }
        }
        ImGui::Separator();
        if (ImGui::Selectable("Rescan mazes", false, ImGuiSelectableFlags_NoAutoClosePopups))
            st.library = scan_library(st.root, st.user_dir);
        ImGui::EndCombo();
    }
}

void source_menu(AppState& st) {
    std::string preview = std::string("Firmware  ") + source_label(st);
    if (st.mms_solver.empty() && !st.firmware.empty() &&
        ImGui::CalcTextSize(preview.c_str()).x > 230.0f - ImGui::GetFrameHeight() * 2.0f)
        preview = "Firmware  " + ui::firmware_info(st.firmware.stem().string()).short_label;
    if (fitted_combo("##src", preview, 230.0f)) {
        if (ImGui::Selectable("No firmware", st.firmware.empty() && st.mms_solver.empty()))
            select_firmware(st, {});
        std::vector<std::string> stems;
        for (const auto& f : st.firmwares) stems.push_back(f.stem().string());
        int shown_group = -1;
        for (const std::size_t i : ui::firmware_menu_order(stems)) {
            const ui::FirmwareInfo fi = ui::firmware_info(stems[i]);
            if (fi.group == 2 && st.mode != ui::Mode::Engineer)
                continue;  // fixtures: Engineer only
            if (fi.group == 2 && shown_group != 2) {
                ImGui::Separator();
                ImGui::TextDisabled("Test fixtures (development)");
            }
            shown_group = fi.group;
            const bool sel = st.firmwares[i] == st.firmware && st.mms_solver.empty();
            if (ImGui::Selectable(fi.label.c_str(), sel)) select_firmware(st, st.firmwares[i]);
            if (ImGui::IsItemHovered(ImGuiHoveredFlags_DelayNormal))
                ImGui::SetTooltip("%s (%s.dll)", fi.blurb.c_str(), stems[i].c_str());
        }
        if (st.firmwares.empty())
            ImGui::TextDisabled("No firmware DLLs found (build firmware_all).");
        if (!st.mms_solver.empty()) {
            ImGui::Separator();
            ImGui::TextDisabled("mms solver: %s", st.mms_solver.filename().string().c_str());
        }
        ImGui::Separator();
        if (ImGui::Selectable("Rescan firmware folder (Ctrl+R)")) st.actions.trigger("reload");
        ImGui::EndCombo();
    }
}

void top_bar(AppState& st) {
    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {12.0f, 8.0f});
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, theme::v4(tok::panel));
    const float h = ImGui::GetFrameHeight() + 16.0f;
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings |
                                   ImGuiWindowFlags_NoScrollWithMouse |
                                   ImGuiWindowFlags_NoDecoration;
    if (ImGui::BeginViewportSideBar("##topbar", vp, ImGuiDir_Up, h, flags)) {
        const ImGuiStyle& s = ImGui::GetStyle();
        const float right_edge = ImGui::GetWindowContentRegionMax().x;
        // Left: brand (dropped first when space is short), maze, source, status.
        static bool hide_brand = false;
        ImGui::PushFont(nullptr, theme::kTitleSize);
        const float brand_w = ImGui::CalcTextSize("MMSim").x + 14.0f;
        if (!hide_brand) {
            ImGui::AlignTextToFramePadding();
            ImGui::TextColored(theme::v4(tok::accent), "MMSim");
            ImGui::SameLine(0, 14.0f);
        }
        ImGui::PopFont();
        maze_menu(st);
        ImGui::SameLine();
        source_menu(st);
        ImGui::SameLine(0, 14.0f);
        const auto ms = current_status(st);
        ui::pill(ms.text.c_str(), ms.tone);
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", ms.detail.c_str());

        // Centre: transport + speed + time.
        ImGui::SameLine(0, 18.0f);
        const bool have = st.session && st.session->state() != host::FwState::LoadError;
        if (ui::action_button("GO", ui::Tone::Good, !have || st.running, 64.0f))
            st.actions.trigger("go");
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Start (Space)");
        ImGui::SameLine();
        if (ui::action_button("STOP", ui::Tone::Bad, !st.running, 64.0f))
            st.actions.trigger("stop");
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Pause (Space)");
        ImGui::SameLine();
        if (ui::action_button("RESET", ui::Tone::Neutral, !st.maze, 0.0f))
            st.actions.trigger("reset");
        if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
            ImGui::SetTooltip("Fresh run, reload firmware (R)");
        ImGui::SameLine(0, 14.0f);
        const int sel = ui::speed_preset_index(st.pacer.speed());
        const auto& presets = ui::speed_presets();
        for (std::size_t i = 0; i < presets.size(); ++i) {
            if (i) ImGui::SameLine(0, 4.0f);
            if (ui::pill_toggle(presets[i].label, sel == static_cast<int>(i),
                                "Simulation speed (+/-)"))
                st.actions.dispatch(std::string("speed ") + presets[i].label);
        }
        ImGui::SameLine(0, 14.0f);
        ImGui::AlignTextToFramePadding();
        ui::mono("%s", ui::format_sim_time(st.session ? st.session->time_us() : 0).c_str());
        if (st.running && st.ratio > 0.0 && !st.pacer.is_max() &&
            st.ratio < 0.97 * st.pacer.speed()) {
            ImGui::SameLine();
            ImGui::TextColored(theme::v4(tok::warn), "%.1fx", st.ratio);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(
                    "Behind real time: the simulation is slower than the chosen speed.");
        }
        const float used =
            ImGui::GetItemRectMax().x - ImGui::GetWindowPos().x + s.ItemSpacing.x * 2;

        // Right: workspace tabs (or a combo when they do not fit), mode switch, help.
        const bool engineer = st.mode == ui::Mode::Engineer;
        std::vector<std::string> tabs;
        std::vector<float> tab_w;
        for (int i = 0; i < ui::kWorkspaceCount; ++i) {
            static const char* kShort[] = {"Cockpit", "Studio", "Telemetry", "Tuning"};
            tabs.push_back(kShort[i]);
            tab_w.push_back(ui::button_width(tabs.back().c_str()));
        }
        const float mode_w = ui::button_width("Beginner") + ui::button_width("Engineer") + 4.0f;
        const float help_w = ui::button_width("?");
        float tabs_total = 0;
        for (float w : tab_w) tabs_total += w + 4.0f;
        const float avail = right_edge - used;
        const float need = tabs_total + mode_w + help_w + s.ItemSpacing.x * 2;
        // Fallbacks: full tabs -> hide the brand -> workspace combo (decided on last frame's
        // widths).
        if (!hide_brand && engineer && need > avail)
            hide_brand = true;
        else if (hide_brand && (!engineer || need + brand_w <= avail))
            hide_brand = false;
        // Narrower tiers: combo with the full title -> combo with the short name -> a single
        // mode toggle instead of the Beginner/Engineer pair.
        const float fp2 = s.FramePadding.x * 2.0f + ImGui::GetFrameHeight() + 8.0f;
        const float combo_full = ImGui::CalcTextSize("Telemetry & Compare").x + fp2;
        const float combo_short = ImGui::CalcTextSize("Telemetry").x + fp2;
        const float mode_one = ui::button_width("Engineer");
        const float sp = s.ItemSpacing.x;
        int tier = 3;
        if (!engineer)
            tier = mode_w + help_w + sp <= avail ? 0 : 3;
        else if (need <= avail)
            tier = 0;
        else if (combo_full + sp + mode_w + help_w + sp <= avail)
            tier = 1;
        else if (combo_short + sp + mode_w + help_w + sp <= avail)
            tier = 2;
        const bool pair = tier < 3;
        const float combo_w = tier == 1 ? combo_full : combo_short;
        const float group_w = (pair ? mode_w : mode_one) + help_w + sp +
                              (engineer ? (tier == 0 ? tabs_total : combo_w) + sp : 0.0f);
        ImGui::SameLine(std::max(used, right_edge - group_w));
        if (engineer) {
            if (tier == 0) {
                for (int i = 0; i < ui::kWorkspaceCount; ++i) {
                    if (i) ImGui::SameLine(0, 4.0f);
                    const std::string tip =
                        std::string(ui::workspace_title(static_cast<ui::Workspace>(i))) +
                        "  (key " + std::to_string(i + 1) + ")";
                    if (ui::pill_toggle(tabs[static_cast<std::size_t>(i)].c_str(),
                                        static_cast<int>(st.workspace) == i, tip.c_str()))
                        st.actions.dispatch("ws " + std::to_string(i + 1));
                }
            } else {
                ImGui::SetNextItemWidth(combo_w);
                const char* preview = tier == 1
                                          ? ui::workspace_title(st.workspace)
                                          : tabs[static_cast<std::size_t>(st.workspace)].c_str();
                if (ImGui::BeginCombo("##ws", preview)) {
                    for (int i = 0; i < ui::kWorkspaceCount; ++i) {
                        const std::string item = std::to_string(i + 1) + "  " +
                                                 ui::workspace_title(static_cast<ui::Workspace>(i));
                        if (ImGui::Selectable(item.c_str(), static_cast<int>(st.workspace) == i))
                            st.actions.dispatch("ws " + std::to_string(i + 1));
                    }
                    ImGui::EndCombo();
                }
                if (ImGui::IsItemHovered()) ImGui::SetTooltip("Workspace (keys 1-4)");
            }
            ImGui::SameLine();
        }
        if (pair) {
            if (ui::pill_toggle("Beginner", !engineer, "Plain language, Cockpit only"))
                st.actions.dispatch("mode beginner");
            ImGui::SameLine(0, 4.0f);
            if (ui::pill_toggle("Engineer", engineer, "All four workspaces and full numbers"))
                st.actions.dispatch("mode engineer");
        } else if (ui::pill_toggle(engineer ? "Engineer" : "Beginner", true,
                                   engineer
                                       ? "Engineer mode. Click for Beginner (Cockpit only)"
                                       : "Beginner mode. Click for Engineer (all workspaces)")) {
            st.actions.dispatch(engineer ? "mode beginner" : "mode engineer");
        }
        ImGui::SameLine();
        if (ui::pill_toggle("?", st.help_open, "Keyboard shortcuts and commands (?)"))
            st.actions.trigger("help");
        // Hairline under the bar.
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImVec2 wp = ImGui::GetWindowPos(), ws = ImGui::GetWindowSize();
        dl->AddLine({wp.x, wp.y + ws.y - 1}, {wp.x + ws.x, wp.y + ws.y - 1},
                    theme::u32(tok::hairline));
    }
    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar(2);
}

// --- console, help, toasts --------------------------------------------------------------------
int console_callback(ImGuiInputTextCallbackData* d) {
    auto* st = static_cast<AppState*>(d->UserData);
    if (d->EventFlag == ImGuiInputTextFlags_CallbackCharFilter) {
        if (d->EventChar == '`') {
            st->console_open = false;
            return 1;
        }
        return 0;
    }
    if (d->EventFlag == ImGuiInputTextFlags_CallbackHistory) {
        const std::string h = st->actions.history_step(d->EventKey == ImGuiKey_UpArrow ? -1 : +1);
        d->DeleteChars(0, d->BufTextLen);
        d->InsertChars(0, h.c_str());
    } else if (d->EventFlag == ImGuiInputTextFlags_CallbackCompletion) {
        const std::string line(d->Buf, static_cast<std::size_t>(d->BufTextLen));
        const std::string done = st->actions.complete_line(line);
        if (done == line) {
            const auto m = st->actions.complete(line);
            if (m.size() > 1) {
                std::string all;
                for (const auto& x : m) all += x + "  ";
                st->console_lines.push_back({all, ui::Tone::Neutral});
            }
        }
        d->DeleteChars(0, d->BufTextLen);
        d->InsertChars(0, done.c_str());
    }
    return 0;
}

void console_window(AppState& st) {
    if (!st.console_open) return;
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    const float w = std::min(vp->WorkSize.x - 24.0f, 760.0f);
    ImGui::SetNextWindowPos({vp->WorkPos.x + (vp->WorkSize.x - w) * 0.5f, vp->WorkPos.y + 8.0f});
    ImGui::SetNextWindowSize({w, 300.0f});
    ImGui::PushStyleColor(ImGuiCol_WindowBg, theme::v4(tok::raised, 0.98f));
    if (ImGui::Begin("Console  (` to close, Tab completes, Up/Down history)", &st.console_open,
                     ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
                         ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoResize |
                         ImGuiWindowFlags_NoMove)) {
        const float input_h = ImGui::GetFrameHeightWithSpacing();
        ImGui::BeginChild("##out", {0, -input_h});
        ImGui::PushFont(theme::fonts().mono, 0.0f);
        for (const auto& [line, tone] : st.console_lines)
            ImGui::TextColored(
                theme::v4(tone == ui::Tone::Neutral ? tok::text : ui::tone_color(tone)), "%s",
                line.c_str());
        ImGui::PopFont();
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4) ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
        static char buf[256] = "";
        ImGui::SetNextItemWidth(-1);
        if (ImGui::IsWindowAppearing()) ImGui::SetKeyboardFocusHere();
        ImGui::PushFont(theme::fonts().mono, 0.0f);
        const bool enter = ImGui::InputTextWithHint(
            "##cmd", "type a command, e.g. speed 5, ws studio, help", buf, sizeof buf,
            ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackHistory |
                ImGuiInputTextFlags_CallbackCompletion | ImGuiInputTextFlags_CallbackCharFilter,
            console_callback, &st);
        ImGui::PopFont();
        if (enter) {
            const std::string line = buf;
            buf[0] = '\0';
            st.console_lines.push_back({"> " + line, ui::Tone::Active});
            st.actions.remember(line);
            const std::string r = st.actions.dispatch(line);
            if (!r.empty())
                st.console_lines.push_back(
                    {r, r.rfind("Unknown", 0) == 0 || r.rfind("usage", 0) == 0
                            ? ui::Tone::Warn
                            : ui::Tone::Neutral});
            ImGui::SetKeyboardFocusHere(-1);
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) st.console_open = false;
    }
    ImGui::End();
    ImGui::PopStyleColor();
    while (st.console_lines.size() > 300) st.console_lines.erase(st.console_lines.begin());
}

void help_modal(AppState& st) {
    if (st.help_open && !ImGui::IsPopupOpen("Keyboard & commands"))
        ImGui::OpenPopup("Keyboard & commands");
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->GetWorkCenter(), ImGuiCond_Appearing, {0.5f, 0.5f});
    ImGui::SetNextWindowSize(
        {std::min(vp->WorkSize.x - 40.0f, 760.0f), std::min(vp->WorkSize.y - 40.0f, 620.0f)},
        ImGuiCond_Appearing);
    bool open = true;
    if (ImGui::BeginPopupModal("Keyboard & commands", &open, ImGuiWindowFlags_NoSavedSettings)) {
        ui::hint("Every shortcut is also a console command (press ` to open the console).");
        const char* groups[] = {"Run", "Workspaces", "View", "Analysis", "App"};
        for (const char* g : groups) {
            ui::section(g);
            if (ImGui::BeginTable(g, 3,
                                  ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg)) {
                ImGui::TableSetupColumn("key", ImGuiTableColumnFlags_WidthFixed, 120.0f);
                ImGui::TableSetupColumn("cmd", ImGuiTableColumnFlags_WidthFixed, 210.0f);
                ImGui::TableSetupColumn("help");
                for (const auto& a : st.actions.actions()) {
                    if (a.group != g) continue;
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    if (!a.keys.empty()) ui::mono_colored(tok::accent, "%s", a.keys.c_str());
                    ImGui::TableNextColumn();
                    ui::mono("%s%s%s", a.name.c_str(), a.usage.empty() ? "" : " ", a.usage.c_str());
                    ImGui::TableNextColumn();
                    ImGui::TextWrapped("%s", a.help.c_str());
                }
                ImGui::EndTable();
            }
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) open = false;
        if (!open) ImGui::CloseCurrentPopup();
        ImGui::EndPopup();
    }
    if (!open) st.help_open = false;
}

void toasts(AppState& st, float dt) {
    const ImGuiViewport* vp = ImGui::GetMainViewport();
    float y = vp->WorkPos.y + vp->WorkSize.y - 16.0f;
    for (auto it = st.toasts.rbegin(); it != st.toasts.rend(); ++it) {
        Toast& t = *it;
        t.age_s += dt;
        const double life = 3.2;
        float a = 1.0f;
        if (theme::motion_enabled()) {
            if (t.age_s < 0.16) a = static_cast<float>(t.age_s / 0.16);
            if (t.age_s > life - 0.2) a = static_cast<float>(std::max(0.0, (life - t.age_s) / 0.2));
        }
        const ImVec2 ts = ImGui::CalcTextSize(t.text.c_str());
        const ImVec2 size{ts.x + 40.0f, ts.y + 18.0f};
        const float slide = (1.0f - a) * 12.0f;
        const ImVec2 p{vp->WorkPos.x + vp->WorkSize.x - size.x - 16.0f, y - size.y + slide};
        ImDrawList* dl = ImGui::GetForegroundDrawList();
        dl->AddRectFilled(p, {p.x + size.x, p.y + size.y}, theme::u32(tok::raised, 0.97f * a),
                          8.0f);
        dl->AddRect(p, {p.x + size.x, p.y + size.y}, theme::u32(tok::hairline, a), 8.0f);
        const Rgba c = t.tone == ui::Tone::Neutral ? tok::muted : ui::tone_color(t.tone);
        dl->AddCircleFilled({p.x + 16.0f, p.y + size.y * 0.5f}, 4.0f, theme::u32(c, a));
        dl->AddText({p.x + 28.0f, p.y + 9.0f}, theme::u32(tok::text, a), t.text.c_str());
        y -= size.y + 8.0f;
    }
    while (!st.toasts.empty() && st.toasts.front().age_s > 3.2) st.toasts.pop_front();
}

// --- actions ----------------------------------------------------------------------------------
void set_workspace(AppState& st, ui::Workspace w) {
    if (st.mode == ui::Mode::Beginner) w = ui::Workspace::Cockpit;
    st.workspace = w;
}

std::string save_screenshot_note(AppState& st) {
    st.want_screenshot = true;
    return "Screenshot queued";
}

void register_actions(AppState& st, Audio& audio) {
    auto& r = st.actions;
    AppState* s = &st;
    r.add({"go", "Space", "Run", "Start the mouse (Space toggles GO / STOP)", "", [s](const auto&) {
               if (!s->session) return std::string("Load a maze first");
               set_running(*s, true);
               return std::string("Running");
           }});
    r.add({"stop", "Space", "Run", "Pause the simulation", "", [s](const auto&) {
               set_running(*s, false);
               return std::string("Paused");
           }});
    r.add({"toggle", "", "Run", "GO if paused, STOP if running", "", [s](const auto&) {
               if (!s->session) return std::string("Load a maze first");
               set_running(*s, !s->running);
               return std::string(s->running ? "Running" : "Paused");
           }});
    r.add({"reset", "R", "Run", "Fresh run: new world, firmware reloaded from disk", "",
           [s](const auto&) {
               make_session(*s);
               return std::string("Reset");
           }});
    r.add(
        {"reload", "Ctrl+R", "Run", "Rescan the firmware folder, then reset", "", [s](const auto&) {
             s->firmwares = scan_firmware(s->firmware_dir);
             make_session(*s);
             return std::string("Firmware rescanned (") + std::to_string(s->firmwares.size()) +
                    " found)";
         }});
    r.add({"step", "Right", "Run", "Advance N ms while paused (Shift+Right = 100 ms)", "<ms>",
           [s](const auto& a) {
               if (!s->session) return std::string("No session");
               const long long ms = a.empty() ? 10 : std::max(1ll, std::atoll(a[0].c_str()));
               set_running(*s, false);
               advance(*s, ms);
               return "Stepped " + std::to_string(ms) + " ms";
           }});
    r.add(
        {"next-cell", "N", "Run", "Run until the mouse enters the next cell", "", [s](const auto&) {
             if (!s->session) return std::string("No session");
             set_running(*s, false);
             advance(*s, 5000, 0.0, true);
             return std::string("Next cell");
         }});
    r.add({"speed", "+ / -", "Run", "Simulation speed", "<1|2|5|fast|x>", [s](const auto& a) {
               if (a.empty()) return std::string("usage: speed <1|2|5|fast|x>");
               std::string v = a[0];
               if (!v.empty() && (v.back() == 'x' || v.back() == 'X')) v.pop_back();
               double sp = 0;
               if (v == "fast" || v == "Fast" || v == "max") {
                   sp = host::Pacer::kMax;
               } else {
                   char* end = nullptr;
                   sp = std::strtod(v.c_str(), &end);
                   if (end == v.c_str() || sp <= 0 || sp > 64)
                       return std::string("usage: speed <1|2|5|fast|x>");
               }
               s->pacer.set_speed(sp);
               s->window_wall = 0;
               s->window_sim_ms = 0;
               return sp <= 0 ? std::string("Speed: fast") : "Speed: " + a[0];
           }});
    auto ladder = [s](bool up) {
        static constexpr double kLadder[] = {0.1, 0.25, 0.5, 1, 2, 5, 10, 20, 64};
        const double cur = s->pacer.is_max() ? 1000.0 : s->pacer.speed();
        double next = up ? host::Pacer::kMax : kLadder[0];
        if (up) {
            for (double l : kLadder)
                if (l > cur + 1e-9) {
                    next = l;
                    break;
                }
        } else {
            for (double l : kLadder)
                if (l < cur - 1e-9) next = l;
        }
        s->pacer.set_speed(next);
        char b[32];
        std::snprintf(b, sizeof b, next <= 0 ? "Speed: fast" : "Speed: %gx", next);
        return std::string(b);
    };
    r.add({"faster", "+", "Run", "Next speed up", "",
           [ladder](const auto&) { return ladder(true); }});
    r.add({"slower", "-", "Run", "Next speed down", "",
           [ladder](const auto&) { return ladder(false); }});
    r.add({"seed", "", "Run", "Noise seed (restarts the run)", "<n>", [s](const auto& a) {
               if (a.empty()) return "Seed " + std::to_string(s->seed);
               s->seed = std::strtoull(a[0].c_str(), nullptr, 0);
               make_session(*s);
               return "Seed " + std::to_string(s->seed) + ", run restarted";
           }});
    r.add(
        {"maze", "", "Run", "Load a maze by (part of its) file name", "<name>", [s](const auto& a) {
             if (a.empty()) return std::string("usage: maze <name>");
             for (const auto& g : s->library)
                 for (const auto& p : g.files)
                     if (p.filename().string().find(a[0]) != std::string::npos) {
                         load_maze(*s, p);
                         return "Loaded " + p.filename().string();
                     }
             return "No maze matches '" + a[0] + "'";
         }});
    r.add({"fw", "", "Run", "Select a firmware DLL by name (none = no firmware)", "<name|none>",
           [s](const auto& a) {
               if (a.empty()) return std::string("Firmware: ") + source_label(*s);
               if (a[0] == "none") {
                   select_firmware(*s, {});
                   return std::string("No firmware");
               }
               std::vector<std::string> stems;
               for (const auto& f : s->firmwares) stems.push_back(f.stem().string());
               if (const auto i = ui::match_firmware(stems, a[0])) {
                   select_firmware(*s, s->firmwares[*i]);
                   return "Firmware " + ui::firmware_info(stems[*i]).label;
               }
               return "No firmware matches '" + a[0] + "'";
           }});
    r.add({"ws", "1 2 3 4", "Workspaces", "Switch workspace: cockpit, studio, telemetry, tuning",
           "<name|1-4>", [s](const auto& a) {
               if (a.empty()) return std::string("usage: ws <cockpit|studio|telemetry|tuning>");
               const auto w = ui::parse_workspace(a[0]);
               if (!w) return "Unknown workspace '" + a[0] + "'";
               if (s->mode == ui::Mode::Beginner && *w != ui::Workspace::Cockpit)
                   return std::string("Beginner mode shows the Cockpit only (mode engineer)");
               set_workspace(*s, *w);
               return std::string(ui::workspace_title(*w));
           }});
    r.add({"mode", "", "Workspaces", "Beginner (plain language) or Engineer (everything)",
           "<beginner|engineer>", [s](const auto& a) {
               const auto m = a.empty() ? std::optional<ui::Mode>(s->mode == ui::Mode::Beginner
                                                                      ? ui::Mode::Engineer
                                                                      : ui::Mode::Beginner)
                                        : ui::parse_mode(a[0]);
               if (!m) return std::string("usage: mode <beginner|engineer>");
               s->mode = *m;
               if (*m == ui::Mode::Beginner) s->workspace = ui::Workspace::Cockpit;
               s->layout_state[static_cast<int>(ui::Workspace::Cockpit)] = 2;  // per-mode layout
               return std::string(*m == ui::Mode::Beginner ? "Beginner mode" : "Engineer mode");
           }});
    r.add({"layout", "", "Workspaces", "Restore the default layout of this workspace", "",
           [s](const auto&) {
               s->layout_state[static_cast<int>(s->workspace)] = 2;
               return std::string("Layout restored");
           }});
    r.add({"fit", "F", "View", "Fit the maze to its panel", "", [s](const auto&) {
               const int w = static_cast<int>(s->workspace);
               s->need_fit[w] = true;
               s->user_moved[w] = false;
               s->iso_pan[w] = {0.0f, 0.0f};
               s->iso_zoom[w] = 1.0f;
               return std::string("");
           }});
    r.add({"iso", "I", "View", "Toggle the 2.5D isometric maze view (Cockpit / Studio)", "", [s](const auto&) {
               s->iso = !s->iso;
               return std::string(s->iso ? "2.5D view" : "2D view");
           }});
    r.add({"heat", "", "View", "Studio heatmap layer", "<off|speed|grip|potential>",
           [s](const auto& a) {
               if (a.empty()) return std::string("usage: heat <off|speed|grip|potential>");
               const std::string v = a[0];
               if (v == "off")
                   s->heat = HeatLayer::Off;
               else if (v == "speed")
                   s->heat = HeatLayer::PeakSpeed;
               else if (v == "grip")
                   s->heat = HeatLayer::GripUse;
               else if (v == "potential")
                   s->heat = HeatLayer::Potential;
               else
                   return std::string("usage: heat <off|speed|grip|potential>");
               return "Heatmap: " + v;
           }});
    r.add({"route", "", "View", "Toggle the planned-route overlay (Studio)", "", [s](const auto&) {
               s->show_route = !s->show_route;
               return std::string(s->show_route ? "Route shown" : "Route hidden");
           }});
    r.add({"screenshot", "F12", "View", "Save a PNG of the window under screenshots/", "",
           [s](const auto&) { return save_screenshot_note(*s); }});
    r.add({"save-run", "", "Analysis", "Archive the current run under runs/ (for Compare)", "",
           [s](const auto&) {
               std::string err;
               const fs::path p = save_current_run(*s, err);
               return p.empty() ? "Save failed: " + err : "Saved " + p.filename().string();
           }});
    r.add({"compare", "", "Analysis", "Pick archived run B by index (0 = newest)", "<index>",
           [s](const auto& a) {
               refresh_archive(*s);
               const int i = a.empty() ? 0 : std::atoi(a[0].c_str());
               return load_compare(*s, i) ? "Run B: " + s->run_b_path.filename().string()
                                          : std::string("No archived run at that index");
           }});
    r.add({"report", "", "Analysis", "Save the offline HTML engineering report under reports/",
           "[file]", [s](const auto& a) {
               const fs::path out = a.empty()
                                        ? timestamped(s->user_dir / "reports", "report", ".html")
                                        : fs::path(a[0]);
               std::string err;
               const bool ok = write_report(*s, out, err);
               s->report_status = ok ? "Saved " + out.string() : err;
               toast(*s, ok ? "Report saved: " + out.filename().string() : "Report failed: " + err,
                     ok ? ui::Tone::Good : ui::Tone::Bad);
               return s->report_status;
           }});
    r.add({"matlab", "", "Analysis", "Export the MATLAB CSV bundle (wheel log + telemetry)", "",
           [s](const auto&) {
               const fs::path dir = timestamped(s->user_dir / "reports", "matlab", "");
               std::string err;
               const bool ok = write_matlab_bundle(*s, dir, err);
               s->report_status = ok ? "Saved " + dir.string() : err;
               toast(*s, ok ? "MATLAB bundle: " + dir.filename().string() : "Export failed: " + err,
                     ok ? ui::Tone::Good : ui::Tone::Bad);
               return s->report_status;
           }});
    r.add({"csv", "", "Analysis", "Export every recorded channel as CSV", "", [s](const auto&) {
               if (!s->session) return std::string("No session");
               const fs::path out = timestamped(s->user_dir / "screenshots", "telemetry", ".csv");
               std::size_t rows = 0;
               const bool ok = export_csv(s->session->recorder(), out, -1e300, 1e300, {}, rows);
               s->report_status =
                   ok ? "Saved " + out.string() + " (" + std::to_string(rows) + " rows)"
                      : "Could not write " + out.string();
               toast(*s, ok ? "CSV saved: " + out.filename().string() : s->report_status,
                     ok ? ui::Tone::Good : ui::Tone::Bad);
               return s->report_status;
           }});
    r.add({"timer", "", "Run", "Race timer: start, pause, reset, or a preset (3, 5, 7 minutes)",
           "<start|pause|reset|3|5|7>", [s](const auto& a) {
               RaceTimer& t = s->timer;
               const std::string v = a.empty() ? "" : a[0];
               if (v == "start") {
                   t.start();
                   s->timer_note.clear();
                   return std::string("Timer running");
               }
               if (v == "pause") {
                   t.pause();
                   return std::string("Timer paused");
               }
               if (v == "reset") {
                   t.reset();
                   s->timer_note.clear();
                   return std::string("Timer reset");
               }
               if (v == "3" || v == "5" || v == "7") {
                   if (t.state() == TimerState::Running || t.state() == TimerState::Paused)
                       return std::string("Reset the timer before changing its length");
                   t.set_preset_minutes(std::atoi(v.c_str()));
                   return "Timer: " + v + " minutes";
               }
               return "Timer " + RaceTimer::format(t.remaining_s()) + " left";
           }});
    r.add({"details", "", "View", "Show / hide the Move & Firmware Details log, or clear it",
           "<show|hide|clear>", [s](const auto& a) {
               const std::string v = a.empty() ? "" : a[0];
               if (v == "clear") {
                   clear_details(*s);
                   return std::string("Details log cleared");
               }
               s->details_open = v == "show" ? true : v == "hide" ? false : !s->details_open;
               return std::string(s->details_open ? "Details log shown" : "Details log hidden");
           }});
    r.add({"mute", "M", "App", "Mute / unmute the goal and crash sounds", "",
           [s, &audio](const auto&) {
               s->muted = !s->muted;
               audio.set_muted(s->muted);
               return std::string(
                   s->muted ? "Sound off"
                            : (audio.available() ? "Sound on" : "Sound on (no audio device)"));
           }});
    r.add({"console", "`", "App", "Open / close this console", "", [s](const auto&) {
               s->console_open = !s->console_open;
               return std::string("");
           }});
    r.add({"help", "?", "App", "Show keyboard shortcuts and commands", "", [s](const auto&) {
               s->help_open = !s->help_open;
               return std::string("");
           }});
    r.add({"clear", "", "App", "Clear the console", "", [s](const auto&) {
               s->console_lines.clear();
               return std::string("");
           }});
}

/// Watches logs / crashes for milestones: toast + sound (never in scripted runs).
void watch_events(AppState& st, Audio& audio) {
    if (!st.session) return;
    const auto& logs = st.session->logs();
    if (st.seen_logs > logs.size()) st.seen_logs = 0;
    for (std::size_t i = st.seen_logs; i < logs.size(); ++i) {
        const std::string& l = logs[i].text;
        if (l.find("speed run done") != std::string::npos) {
            audio.play(Cue::Done);
            toast(st, "Speed run complete", ui::Tone::Good);
        } else if (l.find("goal reached") != std::string::npos) {
            audio.play(Cue::Goal);
            toast(st, "Goal reached!", ui::Tone::Good);
        } else if (ui::classify_log(l) == ui::EventKind::Error) {
            toast(st, l, ui::Tone::Bad);
        }
    }
    st.seen_logs = logs.size();
    const std::size_t crashes = st.session->world().crashes().size();
    if (crashes > st.seen_crashes) {
        static double last = -10.0;
        const double now = ImGui::GetTime();
        if (now - last > 1.5) {
            audio.play(Cue::Crash);
            toast(st, "Wall hit", ui::Tone::Warn);
            last = now;
        }
    }
    st.seen_crashes = crashes;
}

void handle_shortcuts(AppState& st) {
    ImGuiIO& io = ImGui::GetIO();
    if (ImGui::IsKeyPressed(ImGuiKey_F12, false)) st.actions.trigger("screenshot");
    if (io.WantCaptureKeyboard) return;  // typing in a field: no shortcuts
    if (ImGui::IsKeyPressed(ImGuiKey_GraveAccent, false)) st.actions.trigger("console");
    if (ImGui::IsKeyPressed(ImGuiKey_Slash, false) && io.KeyShift) st.actions.trigger("help");
    if (ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId)) return;
    if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) st.actions.trigger("toggle");
    if (ImGui::IsKeyPressed(ImGuiKey_R, false)) st.actions.trigger(io.KeyCtrl ? "reload" : "reset");
    if (ImGui::IsKeyPressed(ImGuiKey_F, false)) st.actions.trigger("fit");
    if (ImGui::IsKeyPressed(ImGuiKey_I, false)) toast(st, st.actions.trigger("iso"));
    if (ImGui::IsKeyPressed(ImGuiKey_N, false)) st.actions.trigger("next-cell");
    if (ImGui::IsKeyPressed(ImGuiKey_M, false)) toast(st, st.actions.trigger("mute"));
    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow) && st.session && !st.running)
        st.actions.dispatch(io.KeyShift ? "step 100" : "step 10");
    if (ImGui::IsKeyPressed(ImGuiKey_Equal, false) ||
        ImGui::IsKeyPressed(ImGuiKey_KeypadAdd, false))
        toast(st, st.actions.trigger("faster"));
    if (ImGui::IsKeyPressed(ImGuiKey_Minus, false) ||
        ImGui::IsKeyPressed(ImGuiKey_KeypadSubtract, false))
        toast(st, st.actions.trigger("slower"));
    const ImGuiKey nums[] = {ImGuiKey_1, ImGuiKey_2, ImGuiKey_3, ImGuiKey_4};
    for (int i = 0; i < 4; ++i)
        if (ImGui::IsKeyPressed(nums[i], false) && !io.KeyShift) {
            const std::string r = st.actions.dispatch("ws " + std::to_string(i + 1));
            if (st.mode == ui::Mode::Beginner && i > 0) toast(st, r);
        }
}

void print_run_summary(const AppState& st) {
    const auto& w = st.session->world();
    const auto& p = w.drive().pose;
    std::printf(
        "run: %s state=%s t=%.3f s pose=(%.4f, %.4f, %.1f deg) crashes=%zu enc=(%ld, %ld)\n",
        st.mms_solver.empty() ? st.firmware.stem().string().c_str()
                              : st.mms_solver.filename().string().c_str(),
        host::to_string(st.session->state()), static_cast<double>(st.session->time_us()) * 1e-6,
        p.x, p.y, p.theta * 180.0 / 3.14159265358979, w.crashes().size(),
        static_cast<long>(w.encoder_count(0)), static_cast<long>(w.encoder_count(1)));
    for (const auto& e : w.crashes())
        std::printf("  crash t=%.4f s speed=%.3f m/s penetration=%.3g mm normal=(%.2f, %.2f)\n",
                    static_cast<double>(e.time_us) * 1e-6, e.impact_speed, e.penetration * 1e3,
                    e.normal.x, e.normal.y);
    for (const auto& l : st.session->logs())
        std::printf("  log %.3f  %s\n", static_cast<double>(l.time_us) * 1e-6, l.text.c_str());
    if (const host::MmsBridge* b = st.session->mms())
        std::printf(
            "mms: running=%d exit=%d cell=(%d, %d) moves=%d crashes=%d invalid=%d score=%s\n",
            st.session->mms_running() ? 1 : 0, st.session->mms_exit_code().value_or(-1),
            b->pose().x / 2, b->pose().y / 2, b->moves(), b->crash_count(), b->invalid_count(),
            b->stats().get("score").c_str());
    std::printf("record: maze=%016llx config=%s firmware=%s seed=%llu\n",
                static_cast<unsigned long long>(w.maze().hash()), st.config_hash.c_str(),
                st.session->firmware_hash().c_str(), static_cast<unsigned long long>(w.seed()));
    std::printf("record: trajectory=%s (t=%.3f s)\n", st.session->trajectory_hash().c_str(),
                static_cast<double>(st.session->time_us()) * 1e-6);
}

}  // namespace

// ---------------------------------------------------------------------------------------------
int main(int argc, char** argv) {
    platform::attach_parent_console();  // GUI-subsystem exe: show --help etc. in a parent terminal
    auto parsed = parse_args(argc, argv);
    if (!parsed) {
        print_usage();
        return 2;
    }
    Args args = *parsed;  // the saved session may fill in what the command line left out

    // Crisp text on high-DPI displays; scripted runs (pixel checks, --size screenshots) stay
    // at 1:1 so a capture is exactly --size on every machine.
    const bool unit_scale = args.verify_render || args.frames > 0;
    SDL_SetHint(SDL_HINT_WINDOWS_DPI_AWARENESS, unit_scale ? "unaware" : "permonitorv2");
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    float scale = unit_scale ? 1.0f : ImGui_ImplSDL2_GetContentScaleForDisplay(0);
    // Interactive windows are created hidden, fitted to the display's work area (whole frame,
    // titlebar included, on screen at any DPI), then shown. Scripted / hidden runs keep the
    // exact --size so captures and pixel checks stay deterministic.
    const bool interactive = !unit_scale && !args.hidden;
    Uint32 wflags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN;
    if (!unit_scale) wflags |= SDL_WINDOW_ALLOW_HIGHDPI;
    SDL_Window* window =
        SDL_CreateWindow("MMSim", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                         static_cast<int>(static_cast<float>(args.width) * scale),
                         static_cast<int>(static_cast<float>(args.height) * scale), wflags);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return 1;
    }
    set_window_icon(window);
    if (!unit_scale) {
        // The window may have opened on a monitor with another DPI than display 0: use the
        // content scale of the display it is really on and resize to the intended logical size.
        const int shown_on = SDL_GetWindowDisplayIndex(window);
        if (shown_on >= 0) {
            const float real = ImGui_ImplSDL2_GetContentScaleForDisplay(shown_on);
            if (real > 0.0f && std::fabs(real - scale) > 1e-3f) {
                scale = real;
                SDL_SetWindowSize(window, static_cast<int>(static_cast<float>(args.width) * scale),
                                  static_cast<int>(static_cast<float>(args.height) * scale));
            }
        }
    }
    if (interactive) {
        const int display = std::max(0, SDL_GetWindowDisplayIndex(window));
        SDL_Rect usable{};
        if (SDL_GetDisplayUsableBounds(display, &usable) == 0) {
            ui::FrameBorders fb;  // Windows 11 fallback when SDL cannot tell yet
            int t = 0, l = 0, b = 0, r = 0;
            if (SDL_GetWindowBordersSize(window, &t, &l, &b, &r) == 0 && t > 0) fb = {t, l, b, r};
            int w = 0, h = 0;
            SDL_GetWindowSize(window, &w, &h);
            const auto fit =
                ui::fit_window(w, h, {usable.x, usable.y, usable.w, usable.h}, fb,
                               static_cast<int>(960.0f * scale), static_cast<int>(600.0f * scale));
            SDL_SetWindowMinimumSize(window, fit.min_w, fit.min_h);
            SDL_SetWindowSize(window, fit.client.w, fit.client.h);
            SDL_SetWindowPosition(window, fit.client.x, fit.client.y);
            if (fit.maximize) SDL_MaximizeWindow(window);
        }
        platform::apply_dark_titlebar(window, 0x0E1116);  // matches tok::canvas
    }
    if (!args.hidden) SDL_ShowWindow(window);
    const Uint32 rflags = args.software ? SDL_RENDERER_SOFTWARE
                                        : (SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, rflags);
    if (!renderer) {
        std::fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    // No keyboard navigation: with it, Space would also "press" the focused button (double
    // toggle) and WantCaptureKeyboard would be true whenever a window has focus.
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    const bool scripted = args.frames > 0;
    // Session persistence (F17): layout + last selections under the data root, never for
    // scripted runs (deterministic layout, no files written by tests).
    const bool persist = !scripted && !args.no_session;
    PathInputs path_in;
    if (args.root) path_in.root_arg = *args.root;
    path_in.portable_flag = args.portable;
    path_in.exe_dir = exe_dir();
    path_in.cwd = fs::current_path();
    if (const char* la = std::getenv("LOCALAPPDATA")) path_in.local_appdata = la;
    const DataPaths data_paths = resolve_data_paths(path_in);
    const fs::path session_root = data_paths.root;
    const fs::path user_dir = data_paths.user_dir;
    if (user_dir != session_root && !scripted) {
        std::error_code mk;
        fs::create_directories(user_dir,
                               mk);  // %LOCALAPPDATA%\MMSim on the first run of an install
    }
    static std::string ini_path;
    ini_path = (user_dir / "imgui.ini").string();
    io.IniFilename = persist ? ini_path.c_str() : nullptr;
    const fs::path settings_file = user_dir / "mmsim.json";
    AppSettings saved;
    const bool cli_started = args.firmware.has_value() || args.mms_solver.has_value();
    if (persist) {
        std::string serr;
        if (!load_settings(settings_file, saved, serr))
            std::fprintf(stderr, "warning: %s (starting with defaults)\n", serr.c_str());
        std::error_code ec;
        const bool any_target = args.maze || args.firmware || args.mms_solver;
        if (!args.maze && !saved.maze.empty() && fs::is_regular_file(saved.maze, ec))
            args.maze = saved.maze;
        if (!args.config && !saved.config.empty() && fs::is_regular_file(saved.config, ec))
            args.config = saved.config;
        if (!any_target) {
            if (!saved.mms_solver.empty() && fs::is_regular_file(saved.mms_solver, ec)) {
                args.mms_solver = saved.mms_solver;
                args.mms_sensor_walls = saved.mms_sensor_walls;
            } else if (!saved.firmware.empty() && fs::is_regular_file(saved.firmware, ec)) {
                args.firmware = saved.firmware.string();
            }
        }
        if (!args.seed_given) args.seed = saved.seed;
        if (!args.speed_given) args.speed = saved.speed;
        if (!args.workspace) args.workspace = saved.workspace;
        if (!args.mode) args.mode = saved.mode;
    }
    theme::apply(scale, exe_dir() / "fonts");
    theme::set_motion_enabled(!scripted);
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    AppState st;
    st.root = session_root;
    st.user_dir = user_dir;
    st.scripted = scripted;
    if (persist) {
        st.boot_buttons[0] = saved.boot_buttons[0];
        st.boot_buttons[1] = saved.boot_buttons[1];
        st.muted = saved.muted;
        st.timer_audio = saved.timer_audio;
        st.details_open = saved.details_open;
        st.timer.set_preset_minutes(saved.timer_minutes);
        st.iso = saved.iso;
        if (saved.layout_version < kLayoutVersion)  // layouts changed in v0.2.0: rebuild once
            for (int& ls : st.layout_state) ls = 2;
    }
    Audio audio;
    audio.init(!scripted && !args.hidden && args.live_for_s <= 0.0);
    audio.set_muted(st.muted);
    register_actions(st, audio);
    if (args.mode)
        if (auto m = ui::parse_mode(*args.mode)) st.mode = *m;
    if (args.workspace)
        if (auto w = ui::parse_workspace(*args.workspace)) set_workspace(st, *w);
    if (args.iso) st.iso = true;
    if (args.verify_render)
        st.workspace = ui::Workspace::Cockpit;  // pixel probes need the plain 2-D maze
    st.seed = args.seed;
    st.pacer.set_speed(args.speed);
    st.library = scan_library(st.root, st.user_dir);
    {
        const fs::path cfg =
            args.config ? *args.config : st.root / "config/robots/generic_classic.json";
        auto r = core::load_robot_config(cfg);
        st.config_path = cfg;
        st.config_hash = file_sha256(cfg);
        if (r.ok()) {
            st.robot = *r.config;
        } else {
            st.robot_error = r.error;
            std::fprintf(stderr, "error: %s\n", r.error.c_str());
            if (args.exit_after) return 1;
        }
    }
    st.firmware_dir = find_firmware_dir(st.root, exe_dir());
    st.firmwares = scan_firmware(st.firmware_dir);
    if (args.mms_solver) {
        st.mms_solver = fs::absolute(*args.mms_solver);
        if (args.mms_sensor_walls) st.mms_opt.walls = host::MmsWallSource::Sensors;
    }
    if (args.firmware) {
        fs::path f = *args.firmware;
        if (f.extension() != ".dll") f = st.firmware_dir / (*args.firmware + ".dll");
        st.firmware = f;
    }
    if (!scripted) {
        if (!args.firmware && !args.mms_solver && st.firmware.empty()) {
            fs::path fallback;
            for (const auto& f : st.firmwares) {
                const std::string stem = f.stem().string();
                if (stem == "demo_floodfill") {
                    st.firmware = f;
                    break;
                }
                if (fallback.empty() && ui::firmware_info(stem).group != 2) fallback = f;
            }
            if (st.firmware.empty() && !fallback.empty()) st.firmware = fallback;
        }
        if (!args.maze) {
            std::error_code ec;
            const fs::path preferred[] = {
                st.root / "mazes/classic/classic_tournament_01.txt",
                st.root / "mazes/test/gen16_loops_a.txt",
            };
            for (const auto& cand : preferred) {
                if (fs::is_regular_file(cand, ec)) {
                    args.maze = cand;
                    break;
                }
            }
            if (!args.maze) {
                for (const auto& g : st.library) {
                    if (!g.files.empty()) {
                        args.maze = g.files.front();
                        break;
                    }
                }
            }
        }
    }
    if (args.maze) {
        load_maze(st, *args.maze);  // also creates the session and loads the firmware
        if (!st.maze) {
            std::fprintf(stderr, "error: %s: %s\n", args.maze->string().c_str(),
                         st.load_error.c_str());
            if (args.exit_after) return 1;
        }
    }
    if ((args.firmware || args.mms_solver) && !st.sim_error.empty()) {
        std::fprintf(stderr, "error: %s\n", st.sim_error.c_str());
        if (args.exit_after) return 1;
    }
    refresh_archive(st);
    if (args.run_for_ms >= 0 && st.session) {
        advance(st, args.run_for_ms);
        if (st.session) {
            print_run_summary(st);
            if (args.export_csv) {
                std::size_t rows = 0;
                if (export_csv(st.session->recorder(), *args.export_csv, -1e300, 1e300, {}, rows)) {
                    std::printf("csv: %s (%zu rows, %zu channels)\n",
                                args.export_csv->string().c_str(), rows,
                                st.session->recorder().channels().size());
                } else {
                    std::fprintf(stderr, "error: could not write %s\n",
                                 args.export_csv->string().c_str());
                    if (args.exit_after) return 1;
                }
            }
            if (args.report) {
                std::string err;
                if (write_report(st, *args.report, err)) {
                    std::printf("report: %s\n", args.report->string().c_str());
                } else {
                    std::fprintf(stderr, "error: report: %s\n", err.c_str());
                    if (args.exit_after) return 1;
                }
            }
            if (args.save_run) {
                std::string err;
                const fs::path p = save_current_run(st, err);
                if (p.empty()) {
                    std::fprintf(stderr, "error: save-run: %s\n", err.c_str());
                    if (args.exit_after) return 1;
                } else {
                    std::printf("run saved: %s\n", p.string().c_str());
                }
            }
            st.tv.follow = true;  // mid-run screenshot: paused, plots show the last window
            st.seen_logs = st.session->logs().size();
            st.seen_crashes = st.session->world().crashes().size();
        } else {
            std::fprintf(stderr, "error: %s\n", st.sim_error.c_str());
            if (args.exit_after) return 1;
        }
    } else if (cli_started && st.session && !scripted) {
        st.running = true;
    }
    // Compare: run B defaults to the newest archived run that is not this one.
    if (!st.archive.empty()) load_compare(st, 0);

    Uint64 last_ticks = SDL_GetPerformanceCounter();
    const Uint64 live_t0 = last_ticks;
    const std::int64_t live_sim0 = st.session ? st.session->time_us() : 0;

    int exit_code = 0;
    bool done = false;
    for (int frame = 0; !done; ++frame) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL2_ProcessEvent(&e);
            if (e.type == SDL_QUIT) done = true;
            if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_CLOSE &&
                e.window.windowID == SDL_GetWindowID(window))
                done = true;
        }
        if (!args.hidden && (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED)) {
            SDL_Delay(10);
            continue;
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        top_bar(st);
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        const ImGuiID dock = ImGui::GetID(dockspace_name(st.workspace));
        const int wi = static_cast<int>(st.workspace);
        if (st.layout_state[wi] != 0) {
            if (st.layout_state[wi] == 2 || scripted || !ImGui::DockBuilderGetNode(dock))
                build_layout(st.workspace, st.mode, dock, vp);
            st.layout_state[wi] = 0;
        }
        ImGui::DockSpaceOverViewport(
            dock, vp, ImGuiDockNodeFlags_NoWindowMenuButton | ImGuiDockNodeFlags_NoCloseButton);

        handle_shortcuts(st);

        // Pacing: simulated time = speed x wall time (exact over many frames, see Pacer);
        // Fast runs as many ticks as fit in a fixed wall budget per frame.
        const Uint64 now_ticks = SDL_GetPerformanceCounter();
        const double wall_dt = static_cast<double>(now_ticks - last_ticks) /
                               static_cast<double>(SDL_GetPerformanceFrequency());
        last_ticks = now_ticks;
        if (st.running && !scripted && st.session) {
            const long long ran = st.pacer.is_max()
                                      ? advance(st, 0, host::Pacer::max_budget_s())
                                      : advance(st, st.pacer.ticks_for_frame(wall_dt));
            st.window_wall += wall_dt;
            st.window_sim_ms += ran;
            if (st.window_wall >= 0.5) {
                st.ratio = static_cast<double>(st.window_sim_ms) / 1000.0 / st.window_wall;
                st.window_wall = 0.0;
                st.window_sim_ms = 0;
            }
        }
        if (!scripted) watch_events(st, audio);
        {
            const TimerCue cue =
                update_race(st, scripted ? 0.0 : static_cast<double>(io.DeltaTime));
            if (!scripted && st.timer_audio) {
                if (cue == TimerCue::Tick) audio.play(Cue::Tick);
                if (cue == TimerCue::TimeUp) audio.play(Cue::TimeUp);
            }
        }

        if (args.live_for_s > 0.0) {
            const double wall = static_cast<double>(now_ticks - live_t0) /
                                static_cast<double>(SDL_GetPerformanceFrequency());
            if (wall >= args.live_for_s) {
                const double sim =
                    st.session ? static_cast<double>(st.session->time_us() - live_sim0) * 1e-6
                               : 0.0;
                std::printf("live: speed %s wall %.3f s sim %.3f s ratio %.4f frames %d\n",
                            st.pacer.is_max() ? "max" : std::to_string(st.pacer.speed()).c_str(),
                            wall, sim, wall > 0.0 ? sim / wall : 0.0, frame);
                if (!st.sim_error.empty()) std::printf("live: error: %s\n", st.sim_error.c_str());
                if (st.session)
                    std::printf("live: firmware %s %s\n", host::to_string(st.session->state()),
                                st.session->error().c_str());
                done = true;
            }
        }

        draw_workspace(st);
        console_window(st);
        help_modal(st);
        toasts(st, io.DeltaTime);

        const std::string title =
            "MMSim " + std::string(core::version()) +
            (st.maze ? " - " + st.maze_path.filename().string() : std::string());
        SDL_SetWindowTitle(window, title.c_str());

        ImGui::Render();
        SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        const auto bg = colors::kBackground;
        SDL_SetRenderDrawColor(renderer, bg.r, bg.g, bg.b, 255);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);

        const bool last_scripted = scripted && frame + 1 >= args.frames;
        if (st.want_screenshot || (last_scripted && (args.screenshot || args.verify_render))) {
            Image img;
            if (!capture_renderer(renderer, img)) {
                std::fprintf(stderr, "error: screenshot capture failed: %s\n", SDL_GetError());
                exit_code = 1;
            } else {
                const fs::path out =
                    (last_scripted && args.screenshot)
                        ? *args.screenshot
                        : timestamped(st.user_dir / "screenshots", "mmsim", ".png");
                if (args.screenshot || st.want_screenshot) {
                    if (write_png(out, img)) {
                        if (!scripted)
                            toast(st, "Screenshot saved: " + out.filename().string(),
                                  ui::Tone::Good);
                        std::printf("screenshot: %s (%dx%d)\n", out.string().c_str(), img.width,
                                    img.height);
                    } else {
                        std::fprintf(stderr, "error: could not write %s\n", out.string().c_str());
                        exit_code = 1;
                    }
                }
                if (last_scripted && args.verify_render) {
                    if (!st.maze) {
                        std::fprintf(stderr, "verify-render: no maze loaded\n");
                        exit_code = 1;
                    } else {
                        const Camera& cam = st.cams[static_cast<int>(ui::Workspace::Cockpit)];
                        auto rc = check_maze_pixels(img, cam, *st.maze, st.dims);
                        std::printf("verify-render: %d probes, %zu failure(s), %.1f px/cell\n",
                                    rc.checked, rc.failures.size(),
                                    st.dims.cell * cam.pixels_per_metre());
                        for (std::size_t i = 0; i < rc.failures.size() && i < 10; ++i)
                            std::printf("  %s\n", rc.failures[i].c_str());
                        if (!rc.ok()) exit_code = 1;
                    }
                }
            }
            st.want_screenshot = false;
        }
        SDL_RenderPresent(renderer);
        if (last_scripted && args.exit_after) done = true;
    }

    audio.shutdown();
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    if (persist) {
        AppSettings now;
        now.maze = st.maze ? fs::absolute(st.maze_path) : fs::path();
        now.config = st.config_path.empty() ? fs::path() : fs::absolute(st.config_path);
        now.firmware = st.firmware.empty() ? fs::path() : fs::absolute(st.firmware);
        now.mms_solver = st.mms_solver;
        now.mms_sensor_walls = st.mms_opt.walls == host::MmsWallSource::Sensors;
        now.seed = st.seed;
        now.speed = st.pacer.speed();
        now.boot_buttons[0] = st.boot_buttons[0];
        now.boot_buttons[1] = st.boot_buttons[1];
        now.workspace = ui::workspace_id(st.workspace);
        now.mode = ui::mode_id(st.mode);
        now.muted = st.muted;
        now.layout_version = kLayoutVersion;
        now.timer_audio = st.timer_audio;
        now.details_open = st.details_open;
        now.timer_minutes = st.timer.preset_minutes();
        now.iso = st.iso;
        std::string serr;
        if (!save_settings(settings_file, now, serr))
            std::fprintf(stderr, "warning: %s\n", serr.c_str());
    }
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return exit_code;
}
