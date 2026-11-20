# Runtime DLLs (libstdc++, SDL2) come from MSYS2; make every test find them without a global PATH change.
set(MMSIM_TEST_ENV "PATH=path_list_prepend:${MSYS2_UCRT64_BIN}")

# --- Unit tests (doctest) ----------------------------------------------------
add_executable(mmsim_unit_tests
    unit/main.cpp
    unit/test_smoke.cpp
    unit/test_maze.cpp
    unit/test_maze_validate.cpp
    unit/test_maze_geometry.cpp
    unit/test_camera.cpp
    unit/test_robot_config.cpp
    unit/test_physics.cpp
    unit/test_collision.cpp
    unit/test_firmware_host.cpp
    unit/test_sensors.cpp
    unit/test_slip.cpp
    unit/test_pacer.cpp
    unit/test_demo_floodfill.cpp)
target_link_libraries(mmsim_unit_tests PRIVATE simcore mmsim_host mmsim_camera mmhal doctest::doctest)
target_compile_definitions(mmsim_unit_tests PRIVATE
    MMSIM_SOURCE_DIR="${PROJECT_SOURCE_DIR}"
    MMSIM_FIRMWARE_DIR="${MMSIM_FIRMWARE_DIR}")
add_dependencies(mmsim_unit_tests firmware_all)
mmsim_set_warnings(mmsim_unit_tests)

add_test(NAME unit.smoke    COMMAND mmsim_unit_tests --test-suite-exclude=maze,app,physics,host,watchdog,sensors,integration)
add_test(NAME unit.maze     COMMAND mmsim_unit_tests --test-suite=maze)
add_test(NAME unit.app      COMMAND mmsim_unit_tests --test-suite=app)
add_test(NAME unit.physics  COMMAND mmsim_unit_tests --test-suite=physics)
add_test(NAME unit.sensors  COMMAND mmsim_unit_tests --test-suite=sensors)
add_test(NAME unit.host     COMMAND mmsim_unit_tests --test-suite=host)
add_test(NAME integration.demo_floodfill COMMAND mmsim_unit_tests --test-suite=integration)
# Own process: a hung firmware leaves a detached spinning thread until exit.
add_test(NAME unit.watchdog COMMAND mmsim_unit_tests --test-suite=watchdog)
set_tests_properties(unit.smoke    PROPERTIES LABELS "unit")
set_tests_properties(unit.maze     PROPERTIES LABELS "unit;maze")
set_tests_properties(unit.app      PROPERTIES LABELS "unit;app")
set_tests_properties(unit.physics  PROPERTIES LABELS "unit;physics")
set_tests_properties(unit.sensors  PROPERTIES LABELS "unit;sensors")
set_tests_properties(unit.host     PROPERTIES LABELS "unit;host")
set_tests_properties(integration.demo_floodfill PROPERTIES LABELS "integration" TIMEOUT 900)
set_tests_properties(unit.watchdog PROPERTIES LABELS "unit;host" TIMEOUT 30)

# --- Visual smoke tests: render headless (software renderer) and pixel-check every wall/post ---
set(MAZEFILES "${PROJECT_SOURCE_DIR}/external/mazefiles")
# Uses the default accelerated renderer = what users see (D3D11; on GPU-less CI runners Windows
# provides WARP). SDL's --software rasterizer is NOT used here: it leaves seams/drops sub-3px
# triangles (see docs/decisions.md D5), which would make pixel checks meaningless.
function(mmsim_render_test name maze size)
    add_test(NAME app.render_${name}
        COMMAND mmsim --maze "${maze}" --hidden --size ${size} --frames 6 --exit ${ARGN}
                --verify-render --screenshot "${CMAKE_BINARY_DIR}/screenshots/${name}.png")
    set_tests_properties(app.render_${name} PROPERTIES LABELS "app;maze;visual"
        ENVIRONMENT_MODIFICATION "${MMSIM_TEST_ENV}")
endfunction()

mmsim_render_test(tiny4x4 "${PROJECT_SOURCE_DIR}/mazes/test/tiny4x4.txt" 1280x800)
if(EXISTS "${MAZEFILES}/classic/uk1987f.txt")
    mmsim_render_test(uk1987f "${MAZEFILES}/classic/uk1987f.txt" 1280x800)
    mmsim_render_test(japan2010hef_32x32 "${MAZEFILES}/halfsize/japan2010hef.txt" 2400x1600)
else()
    message(STATUS "mazefiles not fetched: UK1987/32x32 render tests skipped (run tools/fetch_mazes.ps1)")
endif()

# A broken maze file must make the app exit non-zero.
add_test(NAME app.rejects_bad_maze
    COMMAND mmsim --maze "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/maze/not_enclosed.txt"
            --hidden --frames 1 --exit)
set_tests_properties(app.rejects_bad_maze PROPERTIES WILL_FAIL TRUE LABELS "app;maze"
    ENVIRONMENT_MODIFICATION "${MMSIM_TEST_ENV}")

set_tests_properties(unit.smoke unit.maze unit.app unit.physics unit.sensors unit.host unit.watchdog
    integration.demo_floodfill
    PROPERTIES ENVIRONMENT_MODIFICATION "${MMSIM_TEST_ENV}")

# --- HAL boundary checks (AGENTS.md rules 1-3) -------------------------------
set(BOUNDARY_SCRIPT "${PROJECT_SOURCE_DIR}/tools/check_hal_boundary.py")

# Real firmware tree must be clean.
add_test(NAME boundary.firmware
    COMMAND Python3::Interpreter -I "${BOUNDARY_SCRIPT}" "${PROJECT_SOURCE_DIR}/firmware")

# The HAL header itself may only use C standard headers.
add_test(NAME boundary.hal_header
    COMMAND Python3::Interpreter -I "${BOUNDARY_SCRIPT}" --hal "${PROJECT_SOURCE_DIR}/hal/include")

# Fixtures prove the checker accepts good code and rejects bad code.
add_test(NAME boundary.fixture_good
    COMMAND Python3::Interpreter -I "${BOUNDARY_SCRIPT}" "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/boundary/good")
add_test(NAME boundary.fixture_bad_sim_include
    COMMAND Python3::Interpreter -I "${BOUNDARY_SCRIPT}" "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/boundary/bad_sim_include")
add_test(NAME boundary.fixture_bad_cpp_file
    COMMAND Python3::Interpreter -I "${BOUNDARY_SCRIPT}" "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/boundary/bad_cpp_file")
add_test(NAME boundary.fixture_bad_os_header
    COMMAND Python3::Interpreter -I "${BOUNDARY_SCRIPT}" "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/boundary/bad_os_header")
set_tests_properties(boundary.fixture_bad_sim_include boundary.fixture_bad_cpp_file
    boundary.fixture_bad_os_header PROPERTIES WILL_FAIL TRUE)

set_tests_properties(boundary.firmware boundary.hal_header boundary.fixture_good
    boundary.fixture_bad_sim_include boundary.fixture_bad_cpp_file boundary.fixture_bad_os_header
    PROPERTIES LABELS "boundary")
