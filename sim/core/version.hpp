#pragma once

#include <string_view>

namespace mmsim::core {

/// Simulator version string (CMake project version).
std::string_view version() noexcept;

}  // namespace mmsim::core
