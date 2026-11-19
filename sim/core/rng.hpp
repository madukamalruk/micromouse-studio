#pragma once

#include <cstdint>
#include <string_view>

namespace mmsim::core {

/// splitmix64 step (used to expand seeds).
std::uint64_t splitmix64(std::uint64_t& state);

/// Seed for one named noise source: splitmix64(world_seed ^ FNV-1a(stream)).
/// Every sensor/IMU channel gets its own stream, so adding a sensor never changes the
/// noise of the others, and the same world seed always reproduces the same run.
std::uint64_t derive_seed(std::uint64_t world_seed, std::string_view stream);

/// xoshiro256** 1.0 (Blackman & Vigna), seeded via splitmix64. Deterministic on every platform.
class Rng {
  public:
    Rng() { reseed(1); }
    explicit Rng(std::uint64_t seed) { reseed(seed); }
    void reseed(std::uint64_t seed);
    std::uint64_t next();
    double uniform();  // [0, 1), 53-bit
    double normal();   // standard normal (Box-Muller, no cached spare)

  private:
    std::uint64_t s_[4] = {};
};

}  // namespace mmsim::core
