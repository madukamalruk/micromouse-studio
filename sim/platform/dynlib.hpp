#pragma once

#include <filesystem>
#include <memory>
#include <string>

namespace mmsim::platform {

/// A loaded shared library. open_copy() loads a private temporary copy so that
/// (1) every load gets fresh globals even for the same file (reset, parallel sessions) and
/// (2) the original can be rebuilt while the simulator runs. The copy is deleted on unload.
class DynamicLibrary {
  public:
    static std::unique_ptr<DynamicLibrary> open_copy(const std::filesystem::path& path,
                                                     std::string& error);
    ~DynamicLibrary();
    DynamicLibrary(const DynamicLibrary&) = delete;
    DynamicLibrary& operator=(const DynamicLibrary&) = delete;

    void* symbol(const char* name) const;
    const std::filesystem::path& source() const { return source_; }

  private:
    DynamicLibrary() = default;
    void* module_ = nullptr;
    std::filesystem::path source_;
    std::filesystem::path copy_;
};

}  // namespace mmsim::platform
