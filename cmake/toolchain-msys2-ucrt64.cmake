# Pins the MSYS2 UCRT64 toolchain so the legacy C:\MinGW (GCC 6.3) on PATH is never used.
# Root defaults to D:/msys64; override with the MSYS2_ROOT environment variable (CI sets it).
if(DEFINED ENV{MSYS2_ROOT})
    file(TO_CMAKE_PATH "$ENV{MSYS2_ROOT}" MSYS2_ROOT)
else()
    set(MSYS2_ROOT "D:/msys64")
endif()
set(MSYS2_UCRT64_BIN "${MSYS2_ROOT}/ucrt64/bin")

if(NOT EXISTS "${MSYS2_UCRT64_BIN}/gcc.exe")
    message(FATAL_ERROR "MSYS2 UCRT64 gcc not found at ${MSYS2_UCRT64_BIN}. Set MSYS2_ROOT.")
endif()

set(CMAKE_C_COMPILER   "${MSYS2_UCRT64_BIN}/gcc.exe")
set(CMAKE_CXX_COMPILER "${MSYS2_UCRT64_BIN}/g++.exe")
set(CMAKE_RC_COMPILER  "${MSYS2_UCRT64_BIN}/windres.exe")
set(CMAKE_MAKE_PROGRAM "${MSYS2_UCRT64_BIN}/ninja.exe" CACHE FILEPATH "Ninja from MSYS2")
list(PREPEND CMAKE_PREFIX_PATH "${MSYS2_ROOT}/ucrt64")

# Link the GCC/C++/pthread runtimes statically into every exe/DLL. Otherwise Windows may load
# libstdc++-6.dll from an older toolchain on PATH (e.g. C:\MinGW, GCC 6.3) and crash on ABI mismatch.
set(CMAKE_EXE_LINKER_FLAGS_INIT    "-static-libgcc -static-libstdc++ -Wl,-Bstatic,--whole-archive -lwinpthread -Wl,--no-whole-archive,-Bdynamic")
# Shared libraries = firmware DLLs (pure C, loaded/unloaded on every reset): static libgcc only.
# Whole-archive winpthread would add TLS callbacks to a DLL that is FreeLibrary'd repeatedly.
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-static-libgcc")
set(Python3_ROOT_DIR "${MSYS2_ROOT}/ucrt64")
