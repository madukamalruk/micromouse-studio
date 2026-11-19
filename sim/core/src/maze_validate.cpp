#include "sim/core/maze_validate.hpp"

#include <deque>

namespace mmsim::core {

namespace {

FloodMap bfs(const Maze& m, const std::vector<Cell>& sources) {
    FloodMap f(m.width(), m.height());
    std::deque<Cell> q;
    for (Cell c : sources) {
        if (!m.in_bounds(c) || f.at(c.x, c.y) == 0) continue;
        f.set(c.x, c.y, 0);
        q.push_back(c);
    }
    while (!q.empty()) {
        Cell c = q.front();
        q.pop_front();
        for (Dir d : kAllDirs) {
            if (m.has_wall(c, d)) continue;
            Cell n{c.x + dx(d), c.y + dy(d)};
            if (!m.in_bounds(n) || f.at(n.x, n.y) != FloodMap::kUnreachable) continue;
            f.set(n.x, n.y, f.at(c.x, c.y) + 1);
            q.push_back(n);
        }
    }
    return f;
}

constexpr Dir turn_left(Dir d) { return static_cast<Dir>((static_cast<unsigned>(d) + 3u) & 3u); }
constexpr Dir turn_right(Dir d) { return static_cast<Dir>((static_cast<unsigned>(d) + 1u) & 3u); }

/// Simulates a wall follower; true if it enters a goal cell.
bool wall_follower_solves(const Maze& m, bool left_hand) {
    Cell c = m.start();
    Dir h = m.start_heading();
    const long limit = 4L * m.width() * m.height() * 4L + 16;
    for (long step = 0; step < limit; ++step) {
        if (m.is_goal(c)) return true;
        // Prefer the hand side, then straight, then the other side, then back.
        const Dir side = left_hand ? turn_left(h) : turn_right(h);
        const Dir other = left_hand ? turn_right(h) : turn_left(h);
        const Dir order[4] = {side, h, other, opposite(h)};
        bool moved = false;
        for (Dir d : order) {
            if (!m.has_wall(c, d)) {
                h = d;
                c = {c.x + dx(d), c.y + dy(d)};
                moved = true;
                break;
            }
        }
        if (!moved) return false;  // boxed in
    }
    return false;
}

}  // namespace

FloodMap flood_distances(const Maze& m) { return bfs(m, m.goals()); }

ValidationReport validate(const Maze& m) {
    ValidationReport r;
    r.enclosed = m.is_enclosed();

    const FloodMap from_start = bfs(m, {m.start()});
    for (int y = 0; y < m.height(); ++y)
        for (int x = 0; x < m.width(); ++x)
            if (from_start.at(x, y) == FloodMap::kUnreachable) r.unreachable.push_back({x, y});
    for (Cell g : m.goals())
        if (from_start.at(g.x, g.y) != FloodMap::kUnreachable) r.solvable = true;

    for (Dir d : kAllDirs)
        if (m.has_wall(m.start(), d)) ++r.start_wall_count;

    r.hollow_centre_post = true;
    for (Cell g : m.goals()) {
        for (Dir d : kAllDirs) {
            Cell n{g.x + dx(d), g.y + dy(d)};
            const bool wall = m.has_wall(g, d);
            if (m.in_bounds(n) && m.is_goal(n)) {
                if (wall) r.hollow_centre_post = false;
            } else if (!wall) {
                ++r.goal_entrances;
            }
        }
    }

    // Posts are at lattice points (px, py), px in [0,W], py in [0,H]. A post touches a wall if
    // any of the up-to-4 wall segments meeting at it is present. Posts strictly inside the goal
    // region (all four surrounding cells are goals) are exempt.
    r.all_posts_have_walls = true;
    for (int py = 0; py <= m.height() && r.all_posts_have_walls; ++py) {
        for (int px = 0; px <= m.width(); ++px) {
            auto goal_at = [&](int x, int y) { return m.in_bounds(x, y) && m.is_goal({x, y}); };
            if (goal_at(px - 1, py - 1) && goal_at(px, py - 1) && goal_at(px - 1, py) && goal_at(px, py))
                continue;
            // Post (px,py) is the SW corner of cell (px,py). Segments meeting there:
            // N = W of (px,py) = E of (px-1,py);  S = W of (px,py-1) = E of (px-1,py-1);
            // E = S of (px,py) = N of (px,py-1);  W = S of (px-1,py) = N of (px-1,py-1).
            bool touches = false;
            if (m.in_bounds(px, py) && (m.has_wall(px, py, Dir::W) || m.has_wall(px, py, Dir::S))) touches = true;
            if (m.in_bounds(px - 1, py) && (m.has_wall(px - 1, py, Dir::E) || m.has_wall(px - 1, py, Dir::S)))
                touches = true;
            if (m.in_bounds(px, py - 1) && (m.has_wall(px, py - 1, Dir::W) || m.has_wall(px, py - 1, Dir::N)))
                touches = true;
            if (m.in_bounds(px - 1, py - 1) &&
                (m.has_wall(px - 1, py - 1, Dir::E) || m.has_wall(px - 1, py - 1, Dir::N)))
                touches = true;
            if (!touches) {
                r.all_posts_have_walls = false;
                break;
            }
        }
    }

    r.left_wall_follower_solves = wall_follower_solves(m, true);
    r.right_wall_follower_solves = wall_follower_solves(m, false);
    return r;
}

bool ValidationReport::is_official() const { return issues().empty(); }

std::vector<std::string> ValidationReport::issues() const {
    std::vector<std::string> out;
    if (!enclosed) out.push_back("not enclosed");
    if (!solvable) out.push_back("goal not reachable from start");
    if (!unreachable.empty()) out.push_back(std::to_string(unreachable.size()) + " unreachable cell(s)");
    if (start_wall_count != 3) out.push_back("start cell has " + std::to_string(start_wall_count) + " walls (official: 3)");
    if (goal_entrances != 1) out.push_back("goal has " + std::to_string(goal_entrances) + " entrances (official: 1)");
    if (!hollow_centre_post) out.push_back("walls inside goal region");
    if (!all_posts_have_walls) out.push_back("a post outside the goal has no wall");
    if (left_wall_follower_solves) out.push_back("solvable by left-wall follower");
    if (right_wall_follower_solves) out.push_back("solvable by right-wall follower");
    return out;
}

}  // namespace mmsim::core
