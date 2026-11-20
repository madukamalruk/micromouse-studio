# Camera math: no GUI deps, unit-tested directly.
add_library(mmsim_camera STATIC camera.cpp)
target_include_directories(mmsim_camera PUBLIC "${PROJECT_SOURCE_DIR}")
mmsim_set_warnings(mmsim_camera)

# GUI pieces (ImGui draw lists, screenshots).
add_library(mmsim_applib STATIC maze_view.cpp robot_view.cpp screenshot.cpp telemetry_view.cpp)
target_link_libraries(mmsim_applib PUBLIC mmsim_camera simcore mmsim_host imgui_bundle PRIVATE stb_headers)
mmsim_set_warnings(mmsim_applib)

add_executable(mmsim main.cpp)
target_link_libraries(mmsim PRIVATE mmsim_applib SDL2::SDL2main)
add_dependencies(mmsim firmware_all)  # firmware picker lists build/<preset>/firmware_bin
mmsim_set_warnings(mmsim)

# Make build/<preset>/sim/app/mmsim.exe runnable without MSYS2 on PATH.
add_custom_command(TARGET mmsim POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different $<TARGET_RUNTIME_DLLS:mmsim> $<TARGET_FILE_DIR:mmsim>
    COMMAND_EXPAND_LISTS)
