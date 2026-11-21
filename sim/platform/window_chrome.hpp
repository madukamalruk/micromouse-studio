#pragma once
// Native window chrome (Windows 11 dark titlebar). Win32/DWM stays in sim/platform (D6).

#include <cstdint>

struct SDL_Window;

namespace mmsim::platform {

/// Dark immersive titlebar (DWMWA_USE_IMMERSIVE_DARK_MODE 20, fallback 19 on older
/// Windows 10 builds) and, on Windows 11, the caption colour `rgb` (0xRRGGBB).
/// Returns false when the window has no HWND or DWM refused both attributes.
bool apply_dark_titlebar(SDL_Window* window, std::uint32_t rgb);

/// For the GUI-subsystem exe: when stdout/stderr are not redirected, attach to the parent
/// console (if started from a terminal) so --help and log lines show up there. Never creates a
/// console window, so a Desktop / Start Menu / Explorer launch stays windowless.
void attach_parent_console();

}  // namespace mmsim::platform
