#include "sim/core/analysis/maze_paths.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <deque>
#include <map>
#include <queue>
#include <tuple>

namespace mmsim::core::analysis {

namespace {
constexpr int kDx[4] = {0, 1, 0, -1};
constexpr int kDy[4] = {1, 0, -1, 0};
constexpr std::uint8_t kBit[4] = {1, 2, 4, 8};

int turns_between(int from, int to) {
    if (from == to) return 0;
    return ((to - from) % 4 + 4) % 4 == 2 ? 2 : 1;
}

// One step from (x, y) towards d for shortest_path: only the current cell's knowledge counts
// (dashboard_analysis._open).
bool open_step(const WallGrid& g, int x, int y, int d, bool allow_unknown, int& nx, int& ny) {
    nx = x + kDx[d];
    ny = y + kDy[d];
    if (!g.in(nx, ny)) return false;
    if (g.has_known() && !(g.kn(x, y) & kBit[d])) return allow_unknown;
    return !(g.wall(x, y) & kBit[d]);
}

double round_to(double v, double scale) {
    return std::nearbyint(v * scale) / scale;
}

using State = std::tuple<int, int, int>;  // x, y, heading
using Item = std::pair<double, State>;    // ordered like Python's heap tuples

std::optional<std::vector<RouteStep>> dijkstra(const WallGrid& g, bool allow_unknown,
                                               double turn_cost) {
    const State start{g.start.x, g.start.y, 0};
    std::map<State, double> dist{{start, 0.0}};
    std::map<State, State> prev;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> heap;
    heap.push({0.0, start});
    std::optional<State> goal;
    while (!heap.empty()) {
        const auto [c, s] = heap.top();
        heap.pop();
        const auto it = dist.find(s);
        if (it != dist.end() && c > it->second) continue;
        const auto [x, y, h] = s;
        if (g.is_goal(x, y)) {
            goal = s;
            break;
        }
        for (int d = 0; d < 4; ++d) {
            int nx = 0, ny = 0;
            if (!open_step(g, x, y, d, allow_unknown, nx, ny)) continue;
            const double cost = c + 1.0 + turn_cost * turns_between(h, d);
            const State ns{nx, ny, d};
            const auto f = dist.find(ns);
            if (f == dist.end() || cost < f->second) {
                dist[ns] = cost;
                prev[ns] = s;
                heap.push({cost, ns});
            }
        }
    }
    if (!goal) return std::nullopt;
    std::vector<RouteStep> path;
    State cur = *goal;
    path.push_back({std::get<0>(cur), std::get<1>(cur), std::get<2>(cur)});
    for (auto p = prev.find(cur); p != prev.end(); p = prev.find(cur)) {
        cur = p->second;
        path.push_back({std::get<0>(cur), std::get<1>(cur), std::get<2>(cur)});
    }
    std::reverse(path.begin(), path.end());
    return path;
}
}  // namespace

bool WallGrid::is_goal(int x, int y) const {
    return std::any_of(goals.begin(), goals.end(), [&](Cell c) { return c.x == x && c.y == y; });
}

std::vector<Cell> WallGrid::centre_goals(int w, int h) {
    std::vector<Cell> out;
    const int x0 = (w - 1) / 2, x1 = w / 2, y0 = (h - 1) / 2, y1 = h / 2;
    for (int x : {x0, x1})
        for (int y : {y0, y1})
            if (std::none_of(out.begin(), out.end(), [&](Cell c) { return c.x == x && c.y == y; }))
                out.push_back({x, y});
    return out;
}

WallGrid WallGrid::from_maze(const Maze& m) {
    WallGrid g;
    g.width = m.width();
    g.height = m.height();
    g.walls.resize(static_cast<std::size_t>(g.width * g.height));
    for (int y = 0; y < g.height; ++y)
        for (int x = 0; x < g.width; ++x)
            g.walls[static_cast<std::size_t>(y * g.width + x)] = m.walls(x, y) & 15;
    g.goals = m.goals().empty() ? centre_goals(g.width, g.height) : m.goals();
    g.start = m.start();
    return g;
}

bool side_open(const WallGrid& g, int x, int y, int d, bool optimistic) {
    const int nx = x + kDx[d], ny = y + kDy[d];
    const int back = (d + 2) % 4;
    const bool k_here = !g.has_known() || (g.kn(x, y) & kBit[d]);
    const bool k_there = !g.has_known() || (g.kn(nx, ny) & kBit[back]);
    if (!k_here && !k_there) return optimistic;
    return !((g.wall(x, y) & kBit[d]) || (g.wall(nx, ny) & kBit[back]));
}

std::optional<ShortestPath> shortest_path(const WallGrid& g, double turn_cost) {
    for (const bool allow_unknown : {false, true}) {
        if (allow_unknown && !g.has_known()) break;
        const auto p = dijkstra(g, allow_unknown, turn_cost);
        if (p && !p->empty()) {
            ShortestPath r;
            for (const auto& s : *p) r.cells.push_back({s.x, s.y});
            r.moves = static_cast<int>(r.cells.size()) - 1;
            for (std::size_t i = 1; i < p->size(); ++i)
                r.turns += (*p)[i - 1].heading != (*p)[i].heading;
            r.basis = allow_unknown ? "unknown sides assumed open" : "known open walls only";
            r.optimistic = allow_unknown;
            return r;
        }
    }
    return std::nullopt;
}

DistanceField distance_field(const WallGrid& g, bool optimistic) {
    const auto n = static_cast<std::size_t>(g.width * g.height);
    DistanceField f;
    f.steps.assign(n, -1);
    f.heading.assign(n, -1);
    auto at = [&](int x, int y) { return static_cast<std::size_t>(y * g.width + x); };
    std::deque<Cell> q;
    for (const Cell c : g.goals) {
        f.steps[at(c.x, c.y)] = 0;
        q.push_back(c);
    }
    while (!q.empty()) {
        const Cell c = q.front();
        q.pop_front();
        for (int d = 0; d < 4; ++d) {
            const int nx = c.x + kDx[d], ny = c.y + kDy[d];
            if (!g.in(nx, ny) || f.steps[at(nx, ny)] >= 0) continue;
            if (!side_open(g, c.x, c.y, d, optimistic)) continue;
            f.steps[at(nx, ny)] = f.steps[at(c.x, c.y)] + 1;
            q.push_back({nx, ny});
        }
    }
    for (int y = 0; y < g.height; ++y) {
        for (int x = 0; x < g.width; ++x) {
            const int s = f.steps[at(x, y)];
            if (s > 0) {
                for (int d = 0; d < 4; ++d) {
                    const int nx = x + kDx[d], ny = y + kDy[d];
                    if (g.in(nx, ny) && f.steps[at(nx, ny)] == s - 1 &&
                        side_open(g, x, y, d, optimistic)) {
                        f.heading[at(x, y)] = d;
                        break;
                    }
                }
            }
            const int bits = std::popcount(static_cast<unsigned>(g.wall(x, y) & 15));
            if (!(x == g.start.x && y == g.start.y) && !g.is_goal(x, y) && g.kn(x, y) == 15 &&
                bits == 3)
                f.dead_ends.push_back({x, y});
            f.unreachable += s < 0;
        }
    }
    f.optimistic = optimistic && g.has_known();
    return f;
}

PotentialField potential_field(const WallGrid& g, double turn_cost) {
    std::map<State, double> dist;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> heap;
    for (const Cell c : g.goals) {
        for (int h = 0; h < 4; ++h) {
            dist[{c.x, c.y, h}] = 0.0;
            heap.push({0.0, {c.x, c.y, h}});
        }
    }
    while (!heap.empty()) {
        const auto [c, s] = heap.top();
        heap.pop();
        const auto it = dist.find(s);
        if (it != dist.end() && c > it->second) continue;
        const auto [nx, ny, d] = s;  // arrived in (nx, ny) heading d from one step back
        const int x = nx - kDx[d], y = ny - kDy[d];
        if (!g.in(x, y) || !side_open(g, x, y, d, true)) continue;
        for (int h = 0; h < 4; ++h) {
            const double nc = c + 1.0 + turn_cost * turns_between(d, h);
            const State ps{x, y, h};
            const auto f = dist.find(ps);
            if (f == dist.end() || nc < f->second) {
                dist[ps] = nc;
                heap.push({nc, ps});
            }
        }
    }
    PotentialField p;
    p.turn_cost = turn_cost;
    const auto n = static_cast<std::size_t>(g.width * g.height);
    p.cost.assign(n, -1.0);
    p.heading.assign(n, -1);
    for (int y = 0; y < g.height; ++y) {
        for (int x = 0; x < g.width; ++x) {
            const auto i = static_cast<std::size_t>(y * g.width + x);
            if (g.is_goal(x, y)) {
                p.cost[i] = 0.0;
                continue;
            }
            double best = -1.0;
            int best_d = -1;
            for (int d = 0; d < 4; ++d) {
                const int nx = x + kDx[d], ny = y + kDy[d];
                if (!g.in(nx, ny) || !side_open(g, x, y, d, true)) continue;
                const auto f = dist.find({nx, ny, d});
                if (f != dist.end() && (best_d < 0 || 1.0 + f->second < best)) {
                    best = 1.0 + f->second;
                    best_d = d;
                }
            }
            if (best_d >= 0) {
                p.cost[i] = round_to(best, 100.0);
                p.heading[i] = best_d;
            }
        }
    }
    return p;
}

}  // namespace mmsim::core::analysis
