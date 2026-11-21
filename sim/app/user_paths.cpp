#include "sim/app/user_paths.hpp"

#include <fstream>

namespace mmsim::app {

namespace fs = std::filesystem;

const char* to_string(InstallKind k) {
    switch (k) {
        case InstallKind::Explicit:
            return "explicit";
        case InstallKind::Portable:
            return "portable";
        case InstallKind::DevRepo:
            return "dev";
        case InstallKind::Installed:
            return "installed";
        case InstallKind::Plain:
            return "plain";
        case InstallKind::ReadOnly:
            return "read-only";
    }
    return "?";
}

fs::path find_data_root(const std::vector<fs::path>& starts) {
    for (fs::path p : starts) {
        for (int i = 0; i < 8 && !p.empty(); ++i) {
            std::error_code ec;
            if (fs::is_directory(p / "mazes", ec)) return p;
            if (p == p.parent_path()) break;
            p = p.parent_path();
        }
    }
    return fs::current_path();
}

bool dir_writable(const fs::path& dir) {
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) return false;
    const fs::path probe = dir / ".mmsim_write_probe";
    {
        std::ofstream f(probe, std::ios::binary | std::ios::trunc);
        if (!f) return false;
        f << "x";
        if (!f) return false;
    }
    fs::remove(probe, ec);
    return true;
}

namespace {
bool path_exists(const fs::path& p) {
    std::error_code ec;
    return fs::exists(p, ec);
}
}  // namespace

DataPaths resolve_data_paths(const PathInputs& in,
                             const std::function<bool(const fs::path&)>& writable) {
    DataPaths d;
    if (in.root_arg) {
        d.root = *in.root_arg;
        d.user_dir = d.root;
        d.kind = InstallKind::Explicit;
        return d;
    }
    d.root = find_data_root({in.exe_dir, in.cwd});
    const fs::path appdata = in.local_appdata.empty() ? fs::path{} : in.local_appdata / "MMSim";
    auto user_or_root = [&](InstallKind k) {
        // No %LOCALAPPDATA% (never on a real Windows login): keep state next to the data.
        d.kind = appdata.empty() ? InstallKind::Plain : k;
        d.user_dir = appdata.empty() ? d.root : appdata;
    };

    if (in.portable_flag || (!in.exe_dir.empty() && path_exists(in.exe_dir / kPortableMarker))) {
        d.kind = InstallKind::Portable;
        d.user_dir = d.root;
    } else if (path_exists(d.root / "CMakeLists.txt") && path_exists(d.root / "firmware")) {
        d.kind = InstallKind::DevRepo;
        d.user_dir = d.root;
    } else if (!in.exe_dir.empty() && path_exists(in.exe_dir / kInstalledMarker)) {
        user_or_root(InstallKind::Installed);
    } else if (writable(d.root)) {
        d.kind = InstallKind::Plain;
        d.user_dir = d.root;
    } else {
        user_or_root(InstallKind::ReadOnly);
    }
    return d;
}

}  // namespace mmsim::app
