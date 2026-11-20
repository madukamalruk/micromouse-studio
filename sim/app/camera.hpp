#pragma once

namespace mmsim::app {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

/// 2-D orthographic camera. World: metres, y up. Screen: pixels, y down, inside a viewport.
class Camera {
  public:
    static constexpr double kMinPixelsPerMetre = 20.0;
    static constexpr double kMaxPixelsPerMetre = 40000.0;

    void set_viewport(Vec2 origin, Vec2 size);
    /// Fit the world box [lo, hi] into the viewport with `margin` (fraction of the viewport) per
    /// side.
    void fit(Vec2 lo, Vec2 hi, double margin);
    void zoom_at(Vec2 screen_point, double factor);
    void pan_pixels(Vec2 delta);

    Vec2 world_to_screen(Vec2 w) const;
    Vec2 screen_to_world(Vec2 s) const;
    double pixels_per_metre() const { return scale_; }
    Vec2 viewport_origin() const { return origin_; }
    Vec2 viewport_size() const { return size_; }

  private:
    Vec2 origin_{0, 0};
    Vec2 size_{1, 1};
    Vec2 centre_{0, 0};  // world point at the viewport centre
    double scale_ = 100.0;
};

}  // namespace mmsim::app
