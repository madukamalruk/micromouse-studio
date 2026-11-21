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
add_library(mmsim_uilogic STATIC ui_logic.cpp actions.cpp user_paths.cpp details_log.cpp
    race_timer.cpp help_text.cpp config_edit.cpp)
target_include_directories(mmsim_uilogic PUBLIC "${PROJECT_SOURCE_DIR}")
target_link_libraries(mmsim_uilogic PUBLIC simcore)
mmsim_set_warnings(mmsim_uilogic)

add_library(mmsim_applib STATIC maze_view.cpp robot_view.cpp screenshot.cpp telemetry_view.cpp
    theme.cpp widgets.cpp app_ops.cpp app_ops_diary.cpp maze_canvas.cpp panels_cockpit.cpp
    panels_studio.cpp panels_analysis.cpp panels_tuning.cpp panels_extra.cpp audio.cpp)
target_link_libraries(mmsim_applib PUBLIC mmsim_camera mmsim_settings mmsim_uilogic simcore mmsim_host mmsim_platform_window imgui_bundle
    PRIVATE stb_headers)
mmsim_set_warnings(mmsim_applib)

# Windows resources: icon (id 1, used by SDL for the window and by Explorer) + version block.
set(MMSIM_ICON_FILE "${PROJECT_SOURCE_DIR}/assets/mmsim.ico")
configure_file(mmsim.rc.in "${CMAKE_CURRENT_BINARY_DIR}/mmsim.rc" @ONLY)
add_executable(mmsim main.cpp "${CMAKE_CURRENT_BINARY_DIR}/mmsim.rc")
set_source_files_properties("${CMAKE_CURRENT_BINARY_DIR}/mmsim.rc" PROPERTIES
    OBJECT_DEPENDS "${MMSIM_ICON_FILE}")
target_link_libraries(mmsim PRIVATE mmsim_applib SDL2::SDL2main stb_headers)
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
