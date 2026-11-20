# The ONLY place Win32 APIs are allowed (fibers, LoadLibrary, child processes). See docs/decisions.md D6.
add_library(mmsim_platform STATIC fiber.cpp dynlib.cpp process.cpp)
target_include_directories(mmsim_platform PUBLIC "${PROJECT_SOURCE_DIR}")
mmsim_set_warnings(mmsim_platform)
