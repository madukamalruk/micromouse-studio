#pragma once
// Central catalog of the plain-English (?) tooltips (v0.2.0). The enum and the texts come from
// the same list (help_text.def), so an id can never be missing its text. No GUI dependencies.

#include <cstddef>

namespace mmsim::app {

enum class Help : int {
#define X(id, text) id,
#include "sim/app/help_text.def"
#undef X
    Count
};

/// The tooltip text for an id (never empty).
const char* help_text(Help h);
/// The enumerator name, for tests and diagnostics.
const char* help_name(Help h);

inline constexpr std::size_t kHelpCount = static_cast<std::size_t>(Help::Count);

}  // namespace mmsim::app
