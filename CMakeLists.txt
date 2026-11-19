cmake_minimum_required(VERSION 3.28)
project(mmsim VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)  # no C++20 modules used; avoids per-file scan steps
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(warnings)
include(deps)

find_package(Python3 REQUIRED COMPONENTS Interpreter)

enable_testing()

# --- HAL contract: the ONLY headers firmware may see -------------------------
add_library(mmhal INTERFACE)
target_include_directories(mmhal INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/hal/include")

# --- Simulator ---------------------------------------------------------------
add_subdirectory(sim/core)
add_subdirectory(sim/app)

# --- Tests -------------------------------------------------------------------
add_subdirectory(tests)
