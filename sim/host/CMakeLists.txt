# Firmware host (DLL + fiber lockstep) and Session (World + host = HAL backend).
add_library(mmsim_host STATIC firmware_host.cpp session.cpp guard.cpp harness.cpp pacer.cpp)
target_link_libraries(mmsim_host PUBLIC simcore PRIVATE mmsim_platform mmhal)
mmsim_set_warnings(mmsim_host)
