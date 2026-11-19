/*
 * mmhal/hal.h — the ONLY contract between micromouse firmware and the world.
 *
 * Firmware includes this header and the C standard library, nothing else.
 * The simulator implements it in sim/hal_sim; real hardware implements it in
 * a board backend (Phase 2: hal_stm32.c for Neomouse V2).
 *
 * Slice 1 placeholder: the full API lands in Slice 3 (see docs/plan.md 3.1).
 * Rules: C11 only, no simulator types, compiles for Cortex-M4.
 */
#ifndef MMHAL_HAL_H
#define MMHAL_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MMHAL_VERSION 0

#ifdef __cplusplus
extern "C" {
#endif

/* API is defined in Slice 3. */

#ifdef __cplusplus
}
#endif

#endif /* MMHAL_HAL_H */
