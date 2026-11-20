# Camera math: no GUI deps, unit-tested directly.
add_library(mmsim_camera STATIC camera.cpp)
target_include_directories(mmsim_camera PUBLIC "${PROJECT_SOURCE_DIR}")
mmsim_set_warnings(mmsim_camera)

# GUI pieces (ImGui draw lists, screenshots).
# Session persistence (mmsim.json): no GUI deps, unit-tested directly.
add_library(mmsim_settings STATIC settings.cpp)
target_include_directories(mmsim_settings PUBLIC "${PROJECT_SOURCE_DIR}")
target_link_libraries(mmsim_settings PRIVATE nlohmann_json::nlohmann_json)
mmsim_set_warnings(mmsim_settings)

# Slice 9.10: pure UI logic + the shared action dispatcher (no GUI deps, unit-tested).
add_library(mmsim_uilogic STATIC ui_logic.cpp actions.cpp)
target_include_directories(mmsim_uilogic PUBLIC "${PROJECT_SOURCE_DIR}")
mmsim_set_warnings(mmsim_uilogic)

add_library(mmsim_applib STATIC maze_view.cpp robot_view.cpp screenshot.cpp telemetry_view.cpp
    theme.cpp widgets.cpp app_ops.cpp maze_canvas.cpp panels_cockpit.cpp panels_studio.cpp
    panels_analysis.cpp panels_tuning.cpp audio.cpp)
target_link_libraries(mmsim_applib PUBLIC mmsim_camera mmsim_settings mmsim_uilogic simcore mmsim_host imgui_bundle
    PRIVATE stb_headers)
mmsim_set_warnings(mmsim_applib)

add_executable(mmsim main.cpp)
target_link_libraries(mmsim PRIVATE mmsim_applib SDL2::SDL2main)
add_dependencies(mmsim firmware_all)  # firmware picker lists build/<preset>/firmware_bin
mmsim_set_warnings(mmsim)

# Make build/<preset>/sim/app/mmsim.exe runnable without MSYS2 on PATH.
add_custom_command(TARGET mmsim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:mmsim> $<TARGET_FILE_DIR:mmsim>
    COMMAND_EXPAND_LISTS)
# UI fonts (DESIGN.md): Roboto Medium (Apache-2.0) + Cousine (SIL OFL 1.1), shipped with Dear
# ImGui's sources; copied next to the exe, never committed. Missing -> ImGui's default font.
add_custom_command(TARGET mmsim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:mmsim>/fonts
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${imgui_SOURCE_DIR}/misc/fonts/Roboto-Medium.ttf"
        "${imgui_SOURCE_DIR}/misc/fonts/Cousine-Regular.ttf"
        $<TARGET_FILE_DIR:mmsim>/fonts)
