# Every firmware/<name>/ folder becomes <name>.dll for the simulator: the firmware's own C
# sources + sim/hal_sim/hal_sim.c (the simulator implementation of mmhal/hal.h).
# Firmware sources only see hal/include (checked by tools/check_hal_boundary.py).
set(MMSIM_FIRMWARE_DIR "${CMAKE_BINARY_DIR}/firmware_bin" CACHE INTERNAL "")
set(MMSIM_FIRMWARE_TARGETS "" CACHE INTERNAL "")

function(mmsim_add_firmware name)
    file(GLOB fw_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${name}/*.c")
    # The demo_* solvers share the motion / control / sensing stack in firmware/common.
    if(name MATCHES "^demo_")
        file(GLOB common_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/common/*.c")
        list(APPEND fw_sources ${common_sources})
    endif()
    set(target fw_${name})
    add_library(${target} SHARED ${fw_sources} "${PROJECT_SOURCE_DIR}/sim/hal_sim/hal_sim.c")
    target_link_libraries(${target} PRIVATE mmhal)
    target_include_directories(${target} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")  # "common/x.h"
    # hal_sim.c needs sim/hal_sim/host_api.h; firmware files never include it (boundary test).
    set_source_files_properties("${PROJECT_SOURCE_DIR}/sim/hal_sim/hal_sim.c" TARGET_DIRECTORY ${target}
        PROPERTIES INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}")
    set_target_properties(${target} PROPERTIES
        OUTPUT_NAME ${name} PREFIX ""
        RUNTIME_OUTPUT_DIRECTORY "${MMSIM_FIRMWARE_DIR}"
        LIBRARY_OUTPUT_DIRECTORY "${MMSIM_FIRMWARE_DIR}"
        C_VISIBILITY_PRESET hidden)
    mmsim_set_warnings(${target})
    set(MMSIM_FIRMWARE_TARGETS ${MMSIM_FIRMWARE_TARGETS} ${target} CACHE INTERNAL "")
endfunction()

file(GLOB fw_dirs LIST_DIRECTORIES true RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/*")
foreach(d IN LISTS fw_dirs)
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/${d}" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${d}/main.c")
        mmsim_add_firmware(${d})
    endif()
endforeach()
add_custom_target(firmware_all DEPENDS ${MMSIM_FIRMWARE_TARGETS})
