#include "sim/app/screenshot.hpp"

#include <SDL.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <system_error>

namespace mmsim::app {

bool capture_renderer(SDL_Renderer* renderer, Image& out) {
    int w = 0, h = 0;
    if (SDL_GetRendererOutputSize(renderer, &w, &h) != 0 || w <= 0 || h <= 0) return false;
    out.width = w;
    out.height = h;
    out.rgba.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 4, 0);
    // ABGR8888 is R,G,B,A in memory on little-endian.
    return SDL_RenderReadPixels(renderer, nullptr, SDL_PIXELFORMAT_ABGR8888, out.rgba.data(),
                                w * 4) == 0;
}

bool write_png(const std::filesystem::path& path, const Image& img) {
    std::error_code ec;
    if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path(), ec);
    return stbi_write_png(path.string().c_str(), img.width, img.height, 4, img.rgba.data(),
                          img.width * 4) != 0;
}

}  // namespace mmsim::app
