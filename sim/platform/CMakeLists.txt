# The ONLY place Win32 APIs are allowed (fibers, LoadLibrary, child processes). See docs/decisions.md D6.
add_library(mmsim_platform STATIC fiber.cpp dynlib.cpp process.cpp)
target_include_directories(mmsim_platform PUBLIC "${PROJECT_SOURCE_DIR}")
mmsim_set_warnings(mmsim_platform)

# Native window chrome for the GUI (DWM dark titlebar). Separate so the core platform lib
# (used by the CLI and tests) does not depend on SDL.
add_library(mmsim_platform_window STATIC window_chrome.cpp)
target_include_directories(mmsim_platform_window PUBLIC "${PROJECT_SOURCE_DIR}")
target_link_libraries(mmsim_platform_window PRIVATE SDL2::SDL2 dwmapi)
mmsim_set_warnings(mmsim_platform_window)
