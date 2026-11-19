# --- Unit tests (doctest) ----------------------------------------------------
add_executable(mmsim_unit_tests
    unit/main.cpp
    unit/test_smoke.cpp)
target_link_libraries(mmsim_unit_tests PRIVATE simcore mmhal doctest::doctest)
mmsim_set_warnings(mmsim_unit_tests)

add_test(NAME unit.smoke COMMAND mmsim_unit_tests)
set_tests_properties(unit.smoke PROPERTIES LABELS "unit")

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
