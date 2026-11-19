# arm-check: compile every firmware for Cortex-M4F (STM32F405 class) with arm-none-eabi-gcc,
# against hal/include ONLY. Proves the firmware is portable C11 and touches nothing but the HAL.
# Compile-only: the hardware HAL implementation is Phase 2 (Neomouse V2), so nothing links yet.
#
# Compiler lookup: $ARM_GCC (full path to arm-none-eabi-gcc) -> PATH -> STM32CubeIDE plugin.
set(_arm_hints "")
file(GLOB _cube_bins "C:/ST/STM32CubeIDE*/STM32CubeIDE/plugins/*gnu-tools-for-stm32*/tools/bin")
list(SORT _cube_bins ORDER DESCENDING)  # newest toolchain first
if(DEFINED ENV{ARM_GCC})
    set(MMSIM_ARM_GCC "$ENV{ARM_GCC}" CACHE FILEPATH "arm-none-eabi-gcc for arm-check" FORCE)
else()
    find_program(MMSIM_ARM_GCC arm-none-eabi-gcc HINTS ${_cube_bins})
endif()

set(_arm_flags -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
    -std=c11 -O2 -ffreestanding -Wall -Wextra -Wpedantic -Werror -Wdouble-promotion
    -I${PROJECT_SOURCE_DIR}/hal/include)

if(MMSIM_ARM_GCC)
    message(STATUS "arm-check: ${MMSIM_ARM_GCC}")
    set(_arm_cmds "")
    set(_arm_outs "")
    file(GLOB _fw_c CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/firmware/*/*.c")
    foreach(src IN LISTS _fw_c)
        file(RELATIVE_PATH rel "${PROJECT_SOURCE_DIR}/firmware" "${src}")
        string(REPLACE "/" "_" obj "${rel}")
        set(out "${CMAKE_BINARY_DIR}/arm_check/${obj}.o")
        add_custom_command(OUTPUT "${out}"
            COMMAND "${MMSIM_ARM_GCC}" ${_arm_flags} -c "${src}" -o "${out}"
            DEPENDS "${src}" "${PROJECT_SOURCE_DIR}/hal/include/mmhal/hal.h"
            COMMENT "arm-check ${rel}" VERBATIM)
        list(APPEND _arm_outs "${out}")
    endforeach()
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/arm_check")
    add_custom_target(arm-check DEPENDS ${_arm_outs})
    add_test(NAME boundary.arm_check
        COMMAND ${CMAKE_COMMAND} --build "${CMAKE_BINARY_DIR}" --target arm-check)
    set_tests_properties(boundary.arm_check PROPERTIES LABELS "boundary")
else()
    message(STATUS "arm-check: arm-none-eabi-gcc not found (set ARM_GCC); target will fail")
    add_custom_target(arm-check
        COMMAND ${CMAKE_COMMAND} -E echo "arm-check: arm-none-eabi-gcc not found; set ARM_GCC=<path> and reconfigure"
        COMMAND ${CMAKE_COMMAND} -E false)
endif()
