# mmsim-cli (Slice 8): headless batch runner. Argument parsing is a library so tests use it.
add_library(mmsim_cli_args STATIC cli_args.cpp)
target_include_directories(mmsim_cli_args PUBLIC "${PROJECT_SOURCE_DIR}")
mmsim_set_warnings(mmsim_cli_args)

add_executable(mmsim-cli main.cpp)
target_link_libraries(mmsim-cli PRIVATE mmsim_cli_args mmsim_host simcore)
# Static runtime: runs from any shell without MSYS2 on PATH.
target_link_options(mmsim-cli PRIVATE -static)
add_dependencies(mmsim-cli firmware_all)
mmsim_set_warnings(mmsim-cli)
