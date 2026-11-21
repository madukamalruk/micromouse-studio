#include "sim/core/version.hpp"

namespace mmsim::core {

std::string_view version() noexcept {
    return MMSIM_VERSION;
}

}  // namespace mmsim::core
