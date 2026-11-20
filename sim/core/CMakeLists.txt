add_library(simcore STATIC
    src/version.cpp
    src/maze.cpp
    src/maze_io.cpp
    src/maze_validate.cpp
    src/maze_geometry.cpp
    src/robot_config.cpp
    src/drive.cpp
    src/collision.cpp
    src/world.cpp
    src/rng.cpp
    src/sensors.cpp
    src/sha256.cpp
    analysis/maze_paths.cpp
    analysis/trajectory.cpp
    analysis/metrics.cpp
    analysis/battery.cpp
    analysis/runs.cpp
    analysis/report.cpp)
target_include_directories(simcore PUBLIC "${PROJECT_SOURCE_DIR}")
target_link_libraries(simcore PRIVATE nlohmann_json::nlohmann_json)
target_compile_definitions(simcore PRIVATE MMSIM_VERSION="${PROJECT_VERSION}")
mmsim_set_warnings(simcore)
