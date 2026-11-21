#pragma once
// Where MMSim reads its bundled data and where it writes user state (v0.2.0).
//
//   root      read-only data shipped with the program: mazes/, config/robots/, external/
//   user_dir  writable state: mmsim.json, imgui.ini, runs/, reports/, screenshots/, config copies
//
// Dev checkout, portable zip and explicit --root keep writing next to the data (as before). An
// installed copy (the Setup.exe drops a `mmsim.installed` marker next to the exe) or a program
// folder that cannot be written (Program Files) uses %LOCALAPPDATA%\MMSim instead.

#include <filesystem>
#include <functional>
#include <optional>
#include <vector>

namespace mmsim::app {

inline constexpr const char* kPortableMarker = "mmsim.portable";
inline constexpr const char* kInstalledMarker = "mmsim.installed";

enum class InstallKind {
    Explicit,   // --root DIR: data and state both there
    Portable,   // --portable, or a mmsim.portable marker next to the exe
    DevRepo,    // a source checkout (CMakeLists.txt + firmware/ at the data root)
    Installed,  // mmsim.installed marker next to the exe -> %LOCALAPPDATA%\MMSim
    Plain,      // a loose folder we can write to (unzipped, no marker): state next to the data
    ReadOnly,   // the data folder is not writable -> %LOCALAPPDATA%\MMSim
};

struct PathInputs {
    std::optional<std::filesystem::path> root_arg;  // --root
    bool portable_flag = false;                     // --portable
    std::filesystem::path exe_dir;
    std::filesystem::path cwd;
    std::filesystem::path local_appdata;  // %LOCALAPPDATA%, empty when unset
};

struct DataPaths {
    std::filesystem::path root;
    std::filesystem::path user_dir;
    InstallKind kind = InstallKind::Plain;
};

const char* to_string(InstallKind k);

/// First ancestor (up to 8 levels) of any start folder that contains a `mazes/` directory;
/// the current directory when none does.
std::filesystem::path find_data_root(const std::vector<std::filesystem::path>& starts);

/// True when a file can be created in `dir` (probes with a short-lived file).
bool dir_writable(const std::filesystem::path& dir);

/// Pure policy (the writability probe is injectable for tests).
DataPaths resolve_data_paths(
    const PathInputs& in,
    const std::function<bool(const std::filesystem::path&)>& writable = dir_writable);

}  // namespace mmsim::app
