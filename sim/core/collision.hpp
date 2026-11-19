#pragma once

#include <array>
#include <vector>

#include "sim/core/drive.hpp"
#include "sim/core/maze.hpp"
#include "sim/core/maze_geometry.hpp"
#include "sim/core/robot_config.hpp"

namespace mmsim::core {

/// Oriented box: centre, half extents along its own x (heading) and y, rotation.
struct Obb {
    Vec2d c;
    double half_length = 0.0;
    double half_width = 0.0;
    double theta = 0.0;
};

/// Overlap result. normal = unit direction to move the box out of the obstacle.
struct Contact {
    bool hit = false;
    double depth = 0.0;
    Vec2d normal;
};

Obb body_obb(const Pose& p, const BodyConfig& body);
std::array<Vec2d, 4> obb_corners(const Obb& b);
/// Separating-axis test (4 axes) with minimum translation vector. Touching = no hit.
Contact obb_vs_aabb(const Obb& b, const Rect& r);

struct RayHit {
    bool hit = false;
    double t = 0.0;  // distance along the (unit) direction
    Vec2d normal;    // outward normal of the face that was hit
};

/// Ray vs rectangle (slab method). dir must be unit length. Origin inside -> t = 0.
RayHit ray_vs_aabb(Vec2d origin, Vec2d dir, double tmax, const Rect& r);

/// Static maze obstacles (every wall + every post) in a uniform grid broadphase.
class CollisionWorld {
  public:
    CollisionWorld() = default;
    CollisionWorld(const Maze& m, const MazeDims& d);
    /// Deepest contact of the box against any obstacle.
    Contact query(const Obb& b) const;
    /// Nearest wall/post hit along the ray within tmax.
    RayHit raycast(Vec2d origin, Vec2d dir, double tmax) const;
    std::size_t obstacle_count() const { return rects_.size(); }

  private:
    template <class Fn>
    void for_each_candidate(double minx, double miny, double maxx, double maxy, Fn&& fn) const;

    std::vector<Rect> rects_;
    std::vector<std::vector<int>> grid_;
    double x0_ = 0.0, y0_ = 0.0, cell_ = 1.0;
    int gw_ = 0, gh_ = 0;
    mutable std::vector<unsigned> stamp_;
    mutable unsigned query_id_ = 0;
};

}  // namespace mmsim::core
