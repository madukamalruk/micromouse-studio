#pragma once

#include <filesystem>

namespace test_paths {

inline std::filesystem::path source_dir() { return MMSIM_SOURCE_DIR; }
inline std::filesystem::path mazefiles_dir() { return source_dir() / "external/mazefiles"; }

}  // namespace test_paths
