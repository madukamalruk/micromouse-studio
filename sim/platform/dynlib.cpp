#include "sim/platform/dynlib.hpp"

#include <atomic>
#include <system_error>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace mmsim::platform {

namespace fs = std::filesystem;

std::unique_ptr<DynamicLibrary> DynamicLibrary::open_copy(const fs::path& path,
                                                          std::string& error) {
    static std::atomic<unsigned> counter{0};
    std::error_code ec;
    if (!fs::is_regular_file(path, ec)) {
        error = "firmware not found: " + path.string();
        return nullptr;
    }
    const fs::path dir = fs::temp_directory_path(ec) / "mmsim-fw";
    fs::create_directories(dir, ec);
    const fs::path copy =
        dir / (path.stem().string() + "-" + std::to_string(GetCurrentProcessId()) + "-" +
               std::to_string(counter++) + path.extension().string());
    if (!fs::copy_file(path, copy, fs::copy_options::overwrite_existing, ec)) {
        error = "cannot copy firmware to " + copy.string() + ": " + ec.message();
        return nullptr;
    }
    HMODULE m = LoadLibraryW(copy.wstring().c_str());
    if (!m) {
        error = "LoadLibrary failed for " + path.string() + " (error " +
                std::to_string(GetLastError()) + ")";
        fs::remove(copy, ec);
        return nullptr;
    }
    std::unique_ptr<DynamicLibrary> lib(new DynamicLibrary());
    lib->module_ = m;
    lib->source_ = path;
    lib->copy_ = copy;
    return lib;
}

DynamicLibrary::~DynamicLibrary() {
    if (module_) FreeLibrary(static_cast<HMODULE>(module_));
    std::error_code ec;
    if (!copy_.empty()) fs::remove(copy_, ec);
}

void* DynamicLibrary::symbol(const char* name) const {
    return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(module_), name));
}

}  // namespace mmsim::platform
