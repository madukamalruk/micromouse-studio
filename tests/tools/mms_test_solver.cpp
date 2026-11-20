// Tiny mms-protocol solver used by the Slice 7 tests (stdin/stdout, like mms-cpp).
//   (no args)  flood fill from (0,0) to the centre on the walls it senses; "solver: done" on
//              stderr and exit 0 in the goal, exit 2 on any unexpected reply
//   --echo     echo stdin lines to stdout; "quit" -> "echo: bye" on stderr, exit 3
//   --hang     never read, never exit (kill test)
#include <chrono>
#include <deque>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::string ask(const std::string& cmd) {
    std::cout << cmd << std::endl;
    std::string r;
    if (!(std::cin >> r)) std::exit(4);
    return r;
}
void say(const std::string& cmd) {
    std::cout << cmd << std::endl;
}

constexpr int DX[4] = {0, 1, 0, -1}, DY[4] = {1, 0, -1, 0};
constexpr char DC[4] = {'n', 'e', 's', 'w'};

int solve() {
    const int w = std::stoi(ask("mazeWidth")), h = std::stoi(ask("mazeHeight"));
    auto idx = [w](int x, int y) { return y * w + x; };
    std::vector<int> walls(static_cast<std::size_t>(w * h), 0);  // bit d = wall known present
    auto set_wall = [&](int x, int y, int d) {
        walls[static_cast<std::size_t>(idx(x, y))] |= 1 << d;
        const int nx = x + DX[d], ny = y + DY[d];
        if (nx >= 0 && ny >= 0 && nx < w && ny < h)
            walls[static_cast<std::size_t>(idx(nx, ny))] |= 1 << ((d + 2) % 4);
    };
    auto goal = [&](int x, int y) {
        const bool gx = w % 2 ? x == w / 2 : (x == w / 2 || x == w / 2 - 1);
        const bool gy = h % 2 ? y == h / 2 : (y == h / 2 || y == h / 2 - 1);
        return gx && gy;
    };
    std::vector<int> sent(static_cast<std::size_t>(w * h), -2);
    int x = 0, y = 0, hd = 0;
    for (int steps = 0; steps < 5000; ++steps) {
        const bool f = ask("wallFront") == "true";
        const bool r = ask("wallRight") == "true";
        const bool l = ask("wallLeft") == "true";
        if (f) set_wall(x, y, hd);
        if (r) set_wall(x, y, (hd + 1) % 4);
        if (l) set_wall(x, y, (hd + 3) % 4);
        for (int d = 0; d < 4; ++d)
            if (walls[static_cast<std::size_t>(idx(x, y))] & (1 << d))
                say("setWall " + std::to_string(x) + " " + std::to_string(y) + " " + DC[d]);
        say("setColor " + std::to_string(x) + " " + std::to_string(y) + " B");
        if (goal(x, y)) {
            std::cerr << "solver: done" << std::endl;
            return 0;
        }
        // Flood from the goal (unknown walls open).
        std::vector<int> dist(static_cast<std::size_t>(w * h), -1);
        std::deque<int> q;
        for (int cy = 0; cy < h; ++cy)
            for (int cx = 0; cx < w; ++cx)
                if (goal(cx, cy)) {
                    dist[static_cast<std::size_t>(idx(cx, cy))] = 0;
                    q.push_back(idx(cx, cy));
                }
        while (!q.empty()) {
            const int c = q.front();
            q.pop_front();
            const int cx = c % w, cy = c / w;
            for (int d = 0; d < 4; ++d) {
                if (walls[static_cast<std::size_t>(c)] & (1 << d)) continue;
                const int nx = cx + DX[d], ny = cy + DY[d];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                int& nd = dist[static_cast<std::size_t>(idx(nx, ny))];
                if (nd < 0) {
                    nd = dist[static_cast<std::size_t>(c)] + 1;
                    q.push_back(idx(nx, ny));
                }
            }
        }
        for (int i = 0; i < w * h; ++i)
            if (sent[static_cast<std::size_t>(i)] != dist[static_cast<std::size_t>(i)]) {
                sent[static_cast<std::size_t>(i)] = dist[static_cast<std::size_t>(i)];
                say("setText " + std::to_string(i % w) + " " + std::to_string(i / w) + " " +
                    std::to_string(dist[static_cast<std::size_t>(i)]));
            }
        // Best open neighbour (prefer straight on ties).
        int best = -1, bd = 1 << 30;
        for (int k = 0; k < 4; ++k) {
            const int d = (hd + k) % 4;
            if (walls[static_cast<std::size_t>(idx(x, y))] & (1 << d)) continue;
            const int nx = x + DX[d], ny = y + DY[d];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            const int v = dist[static_cast<std::size_t>(idx(nx, ny))];
            if (v >= 0 && v < bd) {
                bd = v;
                best = d;
            }
        }
        if (best < 0) {
            std::cerr << "solver: no route" << std::endl;
            return 5;
        }
        const int turn = (best - hd + 4) % 4;
        if (turn == 1 && ask("turnRight") != "ack") return 2;
        if (turn == 3 && ask("turnLeft") != "ack") return 2;
        if (turn == 2 && (ask("turnRight") != "ack" || ask("turnRight") != "ack")) return 2;
        hd = best;
        if (ask("moveForward") != "ack") {
            std::cerr << "solver: crashed" << std::endl;
            return 2;
        }
        x += DX[hd];
        y += DY[hd];
    }
    return 6;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string mode = argc > 1 ? argv[1] : "";
    if (mode == "--echo") {
        for (std::string l; std::getline(std::cin, l);) {
            if (!l.empty() && l.back() == '\r') l.pop_back();
            if (l == "quit") {
                std::cerr << "echo: bye" << std::endl;
                return 3;
            }
            std::cout << l << std::endl;
        }
        return 1;
    }
    if (mode == "--hang") {
        for (;;) std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return solve();
}
