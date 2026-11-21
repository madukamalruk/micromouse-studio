#include "sim/platform/window_chrome.hpp"

#include <SDL.h>
#include <SDL_syswm.h>
#include <dwmapi.h>
#include <windows.h>

#include <cstdio>

namespace mmsim::platform {

void attach_parent_console() {
    const auto usable = [](DWORD which) {
        const HANDLE h = GetStdHandle(which);
        return h != nullptr && h != INVALID_HANDLE_VALUE;
    };
    const bool out_ok = usable(STD_OUTPUT_HANDLE), err_ok = usable(STD_ERROR_HANDLE);
    if (out_ok && err_ok) return;  // redirected to a pipe or file: the CRT already uses them
    if (!AttachConsole(ATTACH_PARENT_PROCESS)) return;  // no parent console: stay silent
    if (!out_ok) std::freopen("CONOUT$", "w", stdout);
    if (!err_ok) std::freopen("CONOUT$", "w", stderr);
}

bool apply_dark_titlebar(SDL_Window* window, std::uint32_t rgb) {
    SDL_SysWMinfo info;
    SDL_VERSION(&info.version);
    if (!window || !SDL_GetWindowWMInfo(window, &info) || info.subsystem != SDL_SYSWM_WINDOWS)
        return false;
    HWND hwnd = info.info.win.window;
    constexpr DWORD kDarkMode = 20, kDarkModeOld = 19, kCaptionColor = 35;
    const BOOL dark = TRUE;
    bool ok = SUCCEEDED(DwmSetWindowAttribute(hwnd, kDarkMode, &dark, sizeof dark)) ||
              SUCCEEDED(DwmSetWindowAttribute(hwnd, kDarkModeOld, &dark, sizeof dark));
    // COLORREF is 0x00BBGGRR. Unsupported before Windows 11: ignored.
    const COLORREF caption = RGB((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
    ok = SUCCEEDED(DwmSetWindowAttribute(hwnd, kCaptionColor, &caption, sizeof caption)) || ok;
    return ok;
}

}  // namespace mmsim::platform
