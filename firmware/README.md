# firmware/

One folder per firmware (`firmware/<name>/`), pure C11. Each builds to a DLL for the simulator and must also compile for Cortex-M4 (`arm-check` target, from Slice 3).

Allowed includes: `<mmhal/...>`, C standard headers, and headers inside the same folder. Enforced by `tools/check_hal_boundary.py` (ctest label `boundary`). See AGENTS.md → "HAL boundary rules".
