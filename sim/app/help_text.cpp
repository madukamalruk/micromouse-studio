#include "sim/app/help_text.hpp"

namespace mmsim::app {

namespace {
struct Entry {
    const char* name;
    const char* text;
};
constexpr Entry kEntries[] = {
#define X(id, text) {#id, text},
#include "sim/app/help_text.def"
#undef X
};
static_assert(sizeof(kEntries) / sizeof(kEntries[0]) == kHelpCount,
              "help_text.def entries and the Help enum must stay in step");
}  // namespace

const char* help_text(Help h) {
    const auto i = static_cast<std::size_t>(h);
    return i < kHelpCount ? kEntries[i].text : "";
}

const char* help_name(Help h) {
    const auto i = static_cast<std::size_t>(h);
    return i < kHelpCount ? kEntries[i].name : "?";
}

}  // namespace mmsim::app
