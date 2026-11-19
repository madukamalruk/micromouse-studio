#include "sim/core/rng.hpp"

#include <cmath>
#include <numbers>

namespace mmsim::core {

std::uint64_t splitmix64(std::uint64_t& state) {
    std::uint64_t z = (state += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

std::uint64_t derive_seed(std::uint64_t world_seed, std::string_view stream) {
    std::uint64_t h = 0xcbf29ce484222325ULL;  // FNV-1a 64
    for (unsigned char ch : stream) {
        h ^= ch;
        h *= 0x100000001b3ULL;
    }
    std::uint64_t st = world_seed ^ h;
    return splitmix64(st);
}

void Rng::reseed(std::uint64_t seed) {
    std::uint64_t st = seed;
    for (auto& w : s_) w = splitmix64(st);
}

namespace {
constexpr std::uint64_t rotl(std::uint64_t x, int k) {
    return (x << k) | (x >> (64 - k));
}
}  // namespace

std::uint64_t Rng::next() {
    const std::uint64_t result = rotl(s_[1] * 5, 7) * 9;
    const std::uint64_t t = s_[1] << 17;
    s_[2] ^= s_[0];
    s_[3] ^= s_[1];
    s_[1] ^= s_[2];
    s_[0] ^= s_[3];
    s_[2] ^= t;
    s_[3] = rotl(s_[3], 45);
    return result;
}

double Rng::uniform() {
    return static_cast<double>(next() >> 11) * 0x1.0p-53;
}

double Rng::normal() {
    // Box-Muller; u1 in (0, 1] so log() is finite. One pair per call keeps the stream simple.
    const double u1 = 1.0 - uniform();
    const double u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * std::numbers::pi * u2);
}

}  // namespace mmsim::core
