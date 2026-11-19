#include "sim/app/camera.hpp"

#include <algorithm>

namespace mmsim::app {

void Camera::set_viewport(Vec2 origin, Vec2 size) {
    origin_ = origin;
    size_ = {std::max(1.0, size.x), std::max(1.0, size.y)};
}

void Camera::fit(Vec2 lo, Vec2 hi, double margin) {
    const double w = std::max(1e-9, hi.x - lo.x), h = std::max(1e-9, hi.y - lo.y);
    const double usable = std::max(0.05, 1.0 - 2.0 * margin);
    scale_ = std::clamp(std::min(size_.x * usable / w, size_.y * usable / h), kMinPixelsPerMetre,
                        kMaxPixelsPerMetre);
    centre_ = {(lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0};
}

void Camera::zoom_at(Vec2 screen_point, double factor) {
    const Vec2 anchor = screen_to_world(screen_point);
    scale_ = std::clamp(scale_ * factor, kMinPixelsPerMetre, kMaxPixelsPerMetre);
    // Re-centre so `anchor` maps back to `screen_point`.
    const Vec2 now = screen_to_world(screen_point);
    centre_.x += anchor.x - now.x;
    centre_.y += anchor.y - now.y;
}

void Camera::pan_pixels(Vec2 delta) {
    centre_.x -= delta.x / scale_;
    centre_.y += delta.y / scale_;
}

Vec2 Camera::world_to_screen(Vec2 w) const {
    return {origin_.x + size_.x / 2.0 + (w.x - centre_.x) * scale_,
            origin_.y + size_.y / 2.0 - (w.y - centre_.y) * scale_};
}

Vec2 Camera::screen_to_world(Vec2 s) const {
    return {centre_.x + (s.x - origin_.x - size_.x / 2.0) / scale_,
            centre_.y - (s.y - origin_.y - size_.y / 2.0) / scale_};
}

}  // namespace mmsim::app
