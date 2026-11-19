add_library(simcore STATIC
    src/version.cpp
    src/maze.cpp
    src/maze_io.cpp
    src/maze_validate.cpp
    src/maze_geometry.cpp)
target_include_directories(simcore PUBLIC "${PROJECT_SOURCE_DIR}")
target_compile_definitions(simcore PRIVATE MMSIM_VERSION="${PROJECT_VERSION}")
mmsim_set_warnings(simcore)
