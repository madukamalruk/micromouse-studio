#include <doctest/doctest.h>

#include <string_view>

#include "mmhal/hal.h"
#include "sim/core/version.hpp"

TEST_CASE("simcore reports its version") {
    CHECK(mmsim::core::version() == std::string_view{"0.2.3"});
}

TEST_CASE("HAL header is reachable and versioned") {
    CHECK(MMHAL_VERSION >= 0);
}

TEST_CASE("toolchain is modern (C++20)") {
    CHECK(__cplusplus >= 202002L);
}
