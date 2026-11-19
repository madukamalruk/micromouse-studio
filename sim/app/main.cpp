// MMSim GUI — maze browser, 2-D renderer, firmware runner (SIL, lockstep).
#include <SDL.h>
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <imgui_internal.h>  // DockBuilder
#include <implot.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "sim/app/camera.hpp"
#include "sim/app/maze_view.hpp"
#include "sim/app/robot_view.hpp"
#include "sim/app/screenshot.hpp"
#include "sim/core/maze_io.hpp"
#include "sim/core/maze_validate.hpp"
#include "sim/core/robot_config.hpp"
#include "sim/core/version.hpp"
#include "sim/host/session.hpp"

namespace fs = std::filesystem;
using namespace mmsim;

namespace {

// ---------------------------------------------------------------------------------------------
struct Args {
    std::optional<fs::path> maze;
    std::optional<fs::path> screenshot;
    std::optional<fs::path> root;
    std::optional<std::string> firmware;  // NAME (from the firmware dir) or path to a .dll
    std::optional<fs::path> config;
    long long run_for_ms = -1;  // simulated time to run before the scripted frames
    int frames = -1;            // -1 = run until closed
    bool exit_after = false;
    bool hidden = false;
    bool software = false;
    bool verify_render = false;
    int width = 1280, height = 800;
};

void print_usage() {
    std::printf(
        "mmsim %s — real-time SIL micromouse simulator\n"
        "  --maze FILE          load a maze (.txt map, .num, .maz)\n"
        "  --root DIR           data root containing mazes/ (default: auto-detect)\n"
        "  --firmware NAME|DLL  firmware to run (NAME = build/<preset>/firmware_bin/NAME.dll)\n"
        "  --config FILE        robot config JSON (default config/robots/generic_classic.json)\n"
        "  --run-for T          simulate T (e.g. 3s, 250ms) as fast as possible, then render\n"
        "  --screenshot PNG     save a screenshot of the last frame\n"
        "  --frames N           render N frames\n"
        "  --exit               exit after --frames\n"
        "  --size WxH           window size (default 1280x800)\n"
        "  --hidden             hidden window (with --software for headless checks)\n"
        "  --software           SDL software renderer (diagnostic only; drops thin walls)\n"
        "  --verify-render      pixel-check walls/posts against the maze; exit 1 on mismatch\n",
        std::string(core::version()).c_str());
}

std::optional<Args> parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        std::string s = argv[i];
        auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : nullptr; };
        if (s == "--maze") {
            if (auto v = next())
                a.maze = v;
            else
                return std::nullopt;
        } else if (s == "--screenshot") {
            if (auto v = next())
                a.screenshot = v;
            else
                return std::nullopt;
        } else if (s == "--root") {
            if (auto v = next())
                a.root = v;
            else
                return std::nullopt;
        } else if (s == "--firmware") {
            if (auto v = next())
                a.firmware = v;
            else
                return std::nullopt;
        } else if (s == "--config") {
            if (auto v = next())
                a.config = v;
            else
                return std::nullopt;
        } else if (s == "--run-for") {
            auto v = next();
            if (!v) return std::nullopt;
            char* end = nullptr;
            const double t = std::strtod(v, &end);
            const std::string unit = end ? end : "";
            if (end == v || t < 0 || (!unit.empty() && unit != "s" && unit != "ms"))
                return std::nullopt;
            a.run_for_ms = std::llround(unit == "ms" ? t : t * 1000.0);
        } else if (s == "--frames") {
            if (auto v = next())
                a.frames = std::max(1, std::atoi(v));
            else
                return std::nullopt;
        } else if (s == "--size") {
            auto v = next();
            if (!v || std::sscanf(v, "%dx%d", &a.width, &a.height) != 2) return std::nullopt;
        } else if (s == "--exit") {
            a.exit_after = true;
        } else if (s == "--hidden") {
            a.hidden = true;
        } else if (s == "--software") {
            a.software = true;
        } else if (s == "--verify-render") {
            a.verify_render = true;
        } else {
            return std::nullopt;
        }
    }
    if (a.exit_after && a.frames < 0) a.frames = 5;
    return a;
}

fs::path find_root() {
    std::vector<fs::path> starts;
    if (char* base = SDL_GetBasePath()) {
        starts.emplace_back(base);
        SDL_free(base);
    }
    starts.push_back(fs::current_path());
    for (fs::path p : starts) {
        for (int i = 0; i < 8 && !p.empty(); ++i) {
            if (fs::is_directory(p / "mazes")) return p;
            if (p == p.parent_path()) break;
            p = p.parent_path();
        }
    }
    return fs::current_path();
}

// ---------------------------------------------------------------------------------------------
struct MazeGroup {
    std::string label;
    std::vector<fs::path> files;
};

std::vector<MazeGroup> scan_library(const fs::path& root) {
    std::vector<MazeGroup> groups;
    const std::pair<const char*, fs::path> dirs[] = {
        {"Test mazes", root / "mazes/test"},
        {"Classic 16x16 (mazefiles)", root / "external/mazefiles/classic"},
        {"Half-size (mazefiles)", root / "external/mazefiles/halfsize"},
        {"Training (mazefiles)", root / "external/mazefiles/training"},
    };
    for (const auto& [label, dir] : dirs) {
        MazeGroup g{label, {}};
        std::error_code ec;
        if (fs::is_directory(dir, ec))
            for (const auto& e : fs::directory_iterator(dir, ec)) {
                const auto ext = e.path().extension().string();
                if (ext == ".txt" || ext == ".num" || ext == ".maz") g.files.push_back(e.path());
            }
        std::sort(g.files.begin(), g.files.end());
        groups.push_back(std::move(g));
    }
    return groups;
}

// ---------------------------------------------------------------------------------------------
struct AppState {
    fs::path root;
    std::vector<MazeGroup> library;
    char filter[64] = "";

    std::optional<core::Maze> maze;
    fs::path maze_path;
    std::string load_error;
    std::optional<core::FloodMap> flood;
    core::ValidationReport report;
    core::MazeDims dims;

    app::Camera cam;
    app::MazeViewOptions view;
    bool need_fit = true;
    bool user_moved_camera = false;
    bool want_screenshot = false;
    std::string status;

    // Simulation
    std::optional<core::RobotConfig> robot;
    std::string robot_error;
    fs::path firmware_dir;
    std::vector<fs::path> firmwares;
    fs::path firmware;  // selected DLL
    std::shared_ptr<host::Session> session;
    bool running = false;
    bool boot_buttons[2] = {false, false};
    std::string sim_error;  // load error, hang, ...
    app::PathTrace trace;
    app::RobotViewOptions robot_view;
    bool log_autoscroll = true;
};

fs::path find_firmware_dir(const fs::path& root) {
    std::vector<fs::path> c;
    if (char* base = SDL_GetBasePath()) {
        c.push_back(fs::path(base) / "../../firmware_bin");  // build/<preset>/sim/app/mmsim.exe
        SDL_free(base);
    }
    c.push_back(root / "build/dev/firmware_bin");
    c.push_back(root / "build/release/firmware_bin");
    for (const auto& p : c) {
        std::error_code ec;
        if (fs::is_directory(p, ec)) return fs::weakly_canonical(p, ec);
    }
    return {};
}

std::vector<fs::path> scan_firmware(const fs::path& dir) {
    std::vector<fs::path> out;
    std::error_code ec;
    if (!dir.empty() && fs::is_directory(dir, ec))
        for (const auto& e : fs::directory_iterator(dir, ec))
            if (e.path().extension() == ".dll") out.push_back(e.path());
    std::sort(out.begin(), out.end());
    return out;
}

/// (Re)creates the session for the current maze/robot and loads the selected firmware.
void make_session(AppState& st) {
    st.session.reset();
    st.trace.clear();
    st.running = false;
    st.sim_error.clear();
    if (!st.maze || !st.robot) return;
    st.session = std::make_shared<host::Session>(*st.maze, *st.robot, st.dims);
    st.session->set_button(0, st.boot_buttons[0]);
    st.session->set_button(1, st.boot_buttons[1]);
    if (!st.firmware.empty() && !st.session->load(st.firmware)) st.sim_error = st.session->error();
    st.trace.add(st.session->world().drive().pose);
}

/// Advances the simulation by `ms` ticks on a guarded worker; a firmware that stops calling
/// the HAL (endless loop) is detected and abandoned instead of freezing the GUI.
void advance(AppState& st, long long ms) {
    if (!st.session || ms <= 0) return;
    auto pts = std::make_shared<std::vector<core::Pose>>();
    const auto r = host::run_guarded(
        st.session,
        [ms, pts](host::Session& s) {
            for (long long i = 0; i < ms; ++i) {
                s.step_tick();
                if (s.time_us() % 5000 == 0) pts->push_back(s.world().drive().pose);
            }
        },
        std::chrono::milliseconds(2000 + ms));
    if (r == host::GuardResult::Hung) {
        st.sim_error =
            "Firmware hung: no HAL call for 2 s (endless loop without "
            "hal_delay_ms/hal_wait_tick?). "
            "Session abandoned; press Reset to reload.";
        st.session.reset();
        st.running = false;
        return;
    }
    for (const auto& p : *pts) st.trace.add(p);
    st.trace.add(st.session->world().drive().pose);
}

void load_maze(AppState& st, const fs::path& p) {
    auto r = core::load_maze_file(p);
    st.maze_path = p;
    if (!r.ok()) {
        st.load_error = r.error.to_string();
        return;
    }
    st.load_error.clear();
    st.maze = std::move(*r.maze);
    st.flood = core::flood_distances(*st.maze);
    st.report = core::validate(*st.maze);
    st.need_fit = true;
    st.user_moved_camera = false;
    make_session(st);
}

const char* dir_name(core::Dir d) {
    switch (d) {
        case core::Dir::N:
            return "N";
        case core::Dir::E:
            return "E";
        case core::Dir::S:
            return "S";
        case core::Dir::W:
            return "W";
    }
    return "?";
}

void build_default_layout(ImGuiID dockspace, ImVec2 size) {
    ImGui::DockBuilderRemoveNode(dockspace);
    ImGui::DockBuilderAddNode(dockspace, ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockspace, size);
    ImGuiID centre = dockspace;
    ImGuiID left = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Left, 0.22f, nullptr, &centre);
    ImGuiID right = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Right, 0.26f, nullptr, &centre);
    ImGuiID right_bottom =
        ImGui::DockBuilderSplitNode(right, ImGuiDir_Down, 0.45f, nullptr, &right);
    ImGuiID bottom = ImGui::DockBuilderSplitNode(centre, ImGuiDir_Down, 0.22f, nullptr, &centre);
    ImGui::DockBuilderDockWindow("Mazes", left);
    ImGui::DockBuilderDockWindow("Run", right);
    ImGui::DockBuilderDockWindow("Info", right_bottom);
    ImGui::DockBuilderDockWindow("Log", bottom);
    ImGui::DockBuilderDockWindow("Maze", centre);
    ImGui::DockBuilderFinish(dockspace);
}

void ui_library(AppState& st) {
    ImGui::Begin("Mazes");
    ImGui::SetNextItemWidth(-1);
    ImGui::InputTextWithHint("##filter", "filter (e.g. uk1987)", st.filter, sizeof st.filter);
    const std::string f = st.filter;
    for (const auto& g : st.library) {
        if (g.files.empty()) {
            if (g.label.find("mazefiles") != std::string::npos && g.label.rfind("Classic", 0) == 0)
                ImGui::TextDisabled("mazefiles not fetched: run tools/fetch_mazes.ps1");
            continue;
        }
        if (!ImGui::CollapsingHeader(g.label.c_str(), g.label == "Test mazes" || !f.empty()
                                                          ? ImGuiTreeNodeFlags_DefaultOpen
                                                          : 0))
            continue;
        for (const auto& p : g.files) {
            const std::string name = p.filename().string();
            if (!f.empty() && name.find(f) == std::string::npos) continue;
            if (ImGui::Selectable(name.c_str(), p == st.maze_path)) load_maze(st, p);
        }
    }
    ImGui::End();
}

void ui_info(AppState& st) {
    ImGui::Begin("Info");
    if (!st.load_error.empty()) {
        ImGui::TextColored({1.0f, 0.45f, 0.4f, 1.0f}, "Failed to load %s",
                           st.maze_path.filename().string().c_str());
        ImGui::TextWrapped("%s", st.load_error.c_str());
    }
    if (!st.maze) {
        ImGui::TextDisabled("No maze loaded. Pick one in the Mazes panel.");
        ImGui::End();
        return;
    }
    const auto& m = *st.maze;
    ImGui::SeparatorText("Maze");
    ImGui::Text("File   %s", st.maze_path.filename().string().c_str());
    ImGui::Text("Size   %d x %d", m.width(), m.height());
    ImGui::Text("Start  (%d,%d) facing %s", m.start().x, m.start().y, dir_name(m.start_heading()));
    ImGui::Text("Goals  %zu cell(s)", m.goals().size());
    ImGui::Text("Walls  %d segments", m.wall_count());
    ImGui::Text("Hash   %016llx", static_cast<unsigned long long>(m.hash()));
    if (st.flood)
        ImGui::Text("Start->goal  %d cells (BFS)", st.flood->at(m.start().x, m.start().y));

    ImGui::SeparatorText("Validation (official rules)");
    const auto& r = st.report;
    auto row = [](bool ok, const char* text) {
        ImGui::TextColored(ok ? ImVec4{0.45f, 0.85f, 0.5f, 1} : ImVec4{1.0f, 0.55f, 0.35f, 1},
                           "%s %s", ok ? "[ok]" : "[!!]", text);
    };
    row(r.enclosed, "enclosed");
    row(r.solvable, "goal reachable");
    row(r.unreachable.empty(), "no unreachable cells");
    row(r.start_wall_count == 3, "start has 3 walls");
    row(r.goal_entrances == 1, "single goal entrance");
    row(r.hollow_centre_post, "hollow goal (no inner walls)");
    row(r.all_posts_have_walls, "every post has a wall");
    row(!r.left_wall_follower_solves && !r.right_wall_follower_solves,
        "not wall-follower solvable");
    ImGui::Text("%s", r.is_official() ? "Official contest maze." : "Not an official contest maze.");

    ImGui::SeparatorText("View");
    ImGui::Checkbox("Grid", &st.view.show_grid);
    ImGui::Checkbox("Distances to goal", &st.view.show_distances);
    ImGui::Checkbox("Coordinates", &st.view.show_coords);
    ImGui::Checkbox("Start / goal tint", &st.view.show_start_goal);
    if (ImGui::Button("Fit (F)")) st.need_fit = true;
    ImGui::SameLine();
    if (ImGui::Button("Screenshot (F12)")) st.want_screenshot = true;
    ImGui::TextDisabled("Drag: pan   Wheel: zoom");
    if (!st.status.empty()) ImGui::TextWrapped("%s", st.status.c_str());
    ImGui::End();
}

void ui_maze(AppState& st) {
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0, 0});
    ImGui::Begin("Maze", nullptr,
                 ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();
    const ImVec2 origin = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.x = std::max(avail.x, 50.0f);
    avail.y = std::max(avail.y, 50.0f);
    ImGui::InvisibleButton("canvas", avail,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonMiddle |
                               ImGuiButtonFlags_MouseButtonRight);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    ImGuiIO& io = ImGui::GetIO();

    st.cam.set_viewport({origin.x, origin.y}, {avail.x, avail.y});
    if (st.maze && (st.need_fit || !st.user_moved_camera)) {
        const auto b = core::maze_bounds(*st.maze, st.dims);
        const double pad = st.dims.cell * 0.9;  // room for coordinate labels
        st.cam.fit({b.x0 - pad, b.y0 - pad}, {b.x1 + pad * 0.3, b.y1 + pad * 0.3}, 0.02);
        st.need_fit = false;
    }
    if (active && (io.MouseDelta.x != 0.0f || io.MouseDelta.y != 0.0f)) {
        st.cam.pan_pixels({io.MouseDelta.x, io.MouseDelta.y});
        st.user_moved_camera = true;
    }
    if (hovered && io.MouseWheel != 0.0f) {
        st.cam.zoom_at({io.MousePos.x, io.MousePos.y}, std::pow(1.15, io.MouseWheel));
        st.user_moved_camera = true;
    }
    if ((hovered || ImGui::IsWindowFocused()) && ImGui::IsKeyPressed(ImGuiKey_F, false)) {
        st.need_fit = true;
        st.user_moved_camera = false;
    }

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(origin, {origin.x + avail.x, origin.y + avail.y}, true);
    dl->AddRectFilled(origin, {origin.x + avail.x, origin.y + avail.y},
                      app::colors::kBackground.packed());
    if (st.maze)
        app::draw_maze(dl, st.cam, *st.maze, st.flood ? &*st.flood : nullptr, st.dims, st.view);
    if (st.session) app::draw_robot(dl, st.cam, st.session->world(), st.trace, st.robot_view);
    dl->PopClipRect();
    ImGui::End();
}

void ui_run(AppState& st) {
    ImGui::Begin("Run");
    if (!st.robot) {
        ImGui::TextColored({1.0f, 0.45f, 0.4f, 1.0f}, "Robot config error");
        ImGui::TextWrapped("%s", st.robot_error.c_str());
        ImGui::End();
        return;
    }
    ImGui::SeparatorText("Firmware");
    const std::string current = st.firmware.empty() ? "(none)" : st.firmware.stem().string();
    ImGui::SetNextItemWidth(-1);
    if (ImGui::BeginCombo("##fw", current.c_str())) {
        if (ImGui::Selectable("(none)", st.firmware.empty())) {
            st.firmware.clear();
            make_session(st);
        }
        for (const auto& f : st.firmwares)
            if (ImGui::Selectable(f.stem().string().c_str(), f == st.firmware)) {
                st.firmware = f;
                make_session(st);
            }
        ImGui::EndCombo();
    }
    if (st.firmwares.empty())
        ImGui::TextDisabled("No firmware DLLs found (build target firmware_all).");
    ImGui::Checkbox("Button 0 at boot", &st.boot_buttons[0]);
    ImGui::SameLine();
    ImGui::Checkbox("Button 1", &st.boot_buttons[1]);

    ImGui::SeparatorText("Control");
    const bool have = st.session && st.session->state() != host::FwState::LoadError;
    ImGui::BeginDisabled(!have);
    if (ImGui::Button(st.running ? "Pause (Space)" : "Run (Space)")) st.running = !st.running;
    ImGui::SameLine();
    if (ImGui::Button("Step 1 ms")) {
        st.running = false;
        advance(st, 1);
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Reset (R)")) make_session(st);  // fresh World + DLL reload

    if (!st.sim_error.empty())
        ImGui::TextColored({1.0f, 0.5f, 0.35f, 1.0f}, "%s", st.sim_error.c_str());
    if (!st.session) {
        ImGui::TextDisabled(st.maze ? "No session." : "Load a maze first.");
        ImGui::End();
        return;
    }
    const auto& s = *st.session;
    const auto& w = s.world();
    const auto& d = w.drive();
    ImGui::Text("State   %s", host::to_string(s.state()));
    if (s.state() == host::FwState::Faulted) ImGui::TextWrapped("        %s", s.error().c_str());
    ImGui::Text("Time    %.3f s", static_cast<double>(s.time_us()) * 1e-6);

    ImGui::SeparatorText("Robot");
    ImGui::Text("Pose    x %.4f  y %.4f m  %.1f deg", d.pose.x, d.pose.y,
                d.pose.theta * 180.0 / 3.14159265358979);
    ImGui::Text("Speed   %.3f m/s   yaw %.1f deg/s", w.model().forward_speed(d), w.gyro_z_dps());
    ImGui::Text("Motor   L %.2f V   R %.2f V", w.motor_voltage(0), w.motor_voltage(1));
    if (w.overvoltage_events() > 0)
        ImGui::TextColored({1.0f, 0.75f, 0.3f, 1.0f}, "        clamped to %.1f V (%d times)",
                           w.config().motor.max_voltage, w.overvoltage_events());
    ImGui::Text("Encoder L %ld  R %ld", static_cast<long>(w.encoder_count(0)),
                static_cast<long>(w.encoder_count(1)));
    ImGui::Text("Crashes %zu%s", w.crashes().size(), w.in_contact() ? "  (touching)" : "");
    if (!w.crashes().empty()) {
        const auto& e = w.crashes().back();
        ImGui::TextDisabled("        last at %.3f s, %.2f m/s",
                            static_cast<double>(e.time_us) * 1e-6, e.impact_speed);
    }
    ImGui::Text("LEDs   ");
    for (int i = 0; i < 4; ++i) {
        ImGui::SameLine();
        ImGui::TextColored(s.led(i) ? ImVec4{0.4f, 1.0f, 0.5f, 1} : ImVec4{0.35f, 0.37f, 0.4f, 1},
                           "%s", s.led(i) ? "(*)" : "( )");
    }
    ImGui::Checkbox("Path trace", &st.robot_view.show_trace);
    ImGui::SameLine();
    ImGui::Checkbox("Crash marks", &st.robot_view.show_crashes);
    ImGui::End();
}

void ui_log(AppState& st) {
    ImGui::Begin("Log");
    ImGui::Checkbox("Auto-scroll", &st.log_autoscroll);
    ImGui::Separator();
    ImGui::BeginChild("lines", {0, 0}, ImGuiChildFlags_None, ImGuiWindowFlags_HorizontalScrollbar);
    if (st.session) {
        for (const auto& l : st.session->logs())
            ImGui::Text("%9.3f  %s", static_cast<double>(l.time_us) * 1e-6, l.text.c_str());
        if (st.log_autoscroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 4)
            ImGui::SetScrollHereY(1.0f);
    }
    ImGui::EndChild();
    ImGui::End();
}

fs::path timestamped_screenshot(const fs::path& root) {
    std::time_t t = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof buf, "mmsim-%Y%m%d-%H%M%S.png", std::localtime(&t));
    return root / "screenshots" / buf;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
int main(int argc, char** argv) {
    auto parsed = parse_args(argc, argv);
    if (!parsed) {
        print_usage();
        return 2;
    }
    const Args args = *parsed;

    // Crisp text on high-DPI displays; scripted pixel checks stay at 1:1 scale.
    SDL_SetHint(SDL_HINT_WINDOWS_DPI_AWARENESS, args.verify_render ? "unaware" : "permonitorv2");
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    const float scale = args.verify_render ? 1.0f : ImGui_ImplSDL2_GetContentScaleForDisplay(0);
    Uint32 wflags = SDL_WINDOW_RESIZABLE | (args.hidden ? SDL_WINDOW_HIDDEN : 0);
    if (!args.verify_render) wflags |= SDL_WINDOW_ALLOW_HIGHDPI;
    SDL_Window* window =
        SDL_CreateWindow("MMSim", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                         static_cast<int>(static_cast<float>(args.width) * scale),
                         static_cast<int>(static_cast<float>(args.height) * scale), wflags);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return 1;
    }
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
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard | ImGuiConfigFlags_DockingEnable;
    const bool scripted = args.frames > 0;
    if (scripted) io.IniFilename = nullptr;  // deterministic layout for scripted runs
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 3.0f;
    style.GrabRounding = 3.0f;
    style.ScaleAllSizes(scale);
    style.FontScaleDpi = scale;
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    AppState st;
    st.root = args.root ? *args.root : find_root();
    st.library = scan_library(st.root);
    {
        const fs::path cfg =
            args.config ? *args.config : st.root / "config/robots/generic_classic.json";
        auto r = core::load_robot_config(cfg);
        if (r.ok()) {
            st.robot = *r.config;
        } else {
            st.robot_error = r.error;
            std::fprintf(stderr, "error: %s\n", r.error.c_str());
            if (args.exit_after) return 1;
        }
    }
    st.firmware_dir = find_firmware_dir(st.root);
    st.firmwares = scan_firmware(st.firmware_dir);
    if (args.firmware) {
        fs::path f = *args.firmware;
        if (f.extension() != ".dll") f = st.firmware_dir / (*args.firmware + ".dll");
        st.firmware = f;
    }
    if (args.maze) {
        load_maze(st, *args.maze);  // also creates the session and loads the firmware
        if (!st.maze) {
            std::fprintf(stderr, "error: %s: %s\n", args.maze->string().c_str(),
                         st.load_error.c_str());
            if (args.exit_after) return 1;
        }
    }
    if (args.firmware && !st.sim_error.empty()) {
        std::fprintf(stderr, "error: %s\n", st.sim_error.c_str());
        if (args.exit_after) return 1;
    }
    if (args.run_for_ms >= 0 && st.session) {
        advance(st, args.run_for_ms);
        if (st.session) {
            const auto& w = st.session->world();
            const auto& p = w.drive().pose;
            std::printf(
                "run: %s state=%s t=%.3f s pose=(%.4f, %.4f, %.1f deg) crashes=%zu enc=(%ld, "
                "%ld)\n",
                st.firmware.stem().string().c_str(), host::to_string(st.session->state()),
                static_cast<double>(st.session->time_us()) * 1e-6, p.x, p.y,
                p.theta * 180.0 / 3.14159265358979, w.crashes().size(),
                static_cast<long>(w.encoder_count(0)), static_cast<long>(w.encoder_count(1)));
            for (const auto& e : w.crashes())
                std::printf(
                    "  crash t=%.4f s speed=%.3f m/s penetration=%.3g mm normal=(%.2f, %.2f)\n",
                    static_cast<double>(e.time_us) * 1e-6, e.impact_speed, e.penetration * 1e3,
                    e.normal.x, e.normal.y);
            for (const auto& l : st.session->logs())
                std::printf("  log %.3f  %s\n", static_cast<double>(l.time_us) * 1e-6,
                            l.text.c_str());
        } else {
            std::fprintf(stderr, "error: %s\n", st.sim_error.c_str());
            if (args.exit_after) return 1;
        }
    } else if (args.firmware && st.session && !scripted) {
        st.running = true;
    }
    Uint64 last_ticks = SDL_GetTicks64();

    int exit_code = 0;
    bool done = false;
    bool layout_built = false;
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

        const ImGuiViewport* vp = ImGui::GetMainViewport();
        const ImGuiID dockspace = ImGui::GetID("MainDockSpace");
        if (!layout_built && (scripted || !ImGui::DockBuilderGetNode(dockspace)))
            build_default_layout(dockspace, vp->Size);
        layout_built = true;
        ImGui::DockSpaceOverViewport(dockspace, vp);

        if (ImGui::IsKeyPressed(ImGuiKey_F12, false)) st.want_screenshot = true;
        if (!io.WantTextInput) {
            if (ImGui::IsKeyPressed(ImGuiKey_Space, false) && st.session) st.running = !st.running;
            if (ImGui::IsKeyPressed(ImGuiKey_R, false)) make_session(st);
        }
        // 1x real time (approximate; precise pacing + speed control land in Slice 5).
        const Uint64 now_ticks = SDL_GetTicks64();
        const long long elapsed = static_cast<long long>(now_ticks - last_ticks);
        last_ticks = now_ticks;
        if (st.running && !scripted) advance(st, std::min<long long>(elapsed, 100));

        ui_library(st);
        ui_maze(st);
        ui_run(st);
        ui_info(st);
        ui_log(st);

        const std::string title =
            "MMSim " + std::string(core::version()) +
            (st.maze ? " - " + st.maze_path.filename().string() : std::string());
        SDL_SetWindowTitle(window, title.c_str());

        ImGui::Render();
        SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        const auto bg = app::colors::kBackground;
        SDL_SetRenderDrawColor(renderer, bg.r, bg.g, bg.b, 255);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);

        const bool last_scripted = scripted && frame + 1 >= args.frames;
        if (st.want_screenshot || (last_scripted && (args.screenshot || args.verify_render))) {
            app::Image img;
            if (!app::capture_renderer(renderer, img)) {
                std::fprintf(stderr, "error: screenshot capture failed: %s\n", SDL_GetError());
                exit_code = 1;
            } else {
                fs::path out = (last_scripted && args.screenshot) ? *args.screenshot
                                                                  : timestamped_screenshot(st.root);
                if (args.screenshot || st.want_screenshot) {
                    if (app::write_png(out, img)) {
                        st.status = "Saved " + out.string();
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
                        auto rc = app::check_maze_pixels(img, st.cam, *st.maze, st.dims);
                        std::printf("verify-render: %d probes, %zu failure(s), %.1f px/cell\n",
                                    rc.checked, rc.failures.size(),
                                    st.dims.cell * st.cam.pixels_per_metre());
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

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return exit_code;
}
