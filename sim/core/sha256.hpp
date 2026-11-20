#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

namespace mmsim::core {

/// SHA-256 (FIPS 180-4), incremental. Used for run records: maze/config/firmware hashes and
/// the per-tick trajectory hash that proves two runs were identical.
class Sha256 {
  public:
    Sha256();
    void update(const void* data, std::size_t len);
    void update(std::string_view s) { update(s.data(), s.size()); }
    /// Digest of everything so far; the object stays usable (works on a copy).
    std::array<std::uint8_t, 32> digest() const;
    std::string hex() const;

  private:
    void block(const std::uint8_t* p);
    std::uint32_t h_[8];
    std::uint8_t buf_[64];
    std::size_t buf_len_ = 0;
    std::uint64_t total_ = 0;
};

std::string sha256_hex(std::string_view data);

}  // namespace mmsim::core
