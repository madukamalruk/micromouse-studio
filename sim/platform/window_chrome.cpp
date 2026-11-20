#include "sim/platform/window_chrome.hpp"

#include <SDL.h>
#include <SDL_syswm.h>
#include <dwmapi.h>
#include <windows.h>

namespace mmsim::platform {

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
