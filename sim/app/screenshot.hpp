#pragma once

#include <filesystem>

#include "sim/app/maze_view.hpp"

struct SDL_Renderer;

namespace mmsim::app {

/// Reads the current render target (call after drawing, before SDL_RenderPresent).
bool capture_renderer(SDL_Renderer* renderer, Image& out);

/// Writes RGBA8 image as PNG. Creates parent directories.
bool write_png(const std::filesystem::path& path, const Image& img);

}  // namespace mmsim::app
