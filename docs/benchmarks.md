# Solver benchmark

Five demo solvers on the generic classic robot (`config/robots/generic_classic.json`), noise
seed 1, simulated seconds, produced by `python tools/benchmark.py` (release build, `mmsim-cli`).
Every run is deterministic: the same command gives the same table.

- **Search** = start to goal. **Return** = goal back to start. **Speed run** = the final fast
  run over the known shortest route. Lower is better.
- `gen16_*` are the original test mazes in `mazes/test/` (made by `tools/gen_mazes.py`):
  `perfect` mazes are spanning trees (the goal touches the outer wall), `loops` mazes also have
  extra openings, like contest mazes.
- `uk1987f` comes from the classic maze collection and appears only if you fetched it with
  `tools/fetch_mazes.ps1` (it is not part of this repository).

| Maze | Algorithm | Result | Search | Return | Speed run | Crashes |
|---|---|---|---:|---:|---:|---:|
| gen16_loops_a | Flood-Fill | solved | 27.2 s | 37.8 s | **19.9 s** | 0 |
| gen16_loops_a | Turn-Weighted Dijkstra | solved | 27.2 s | 31.9 s | **19.7 s** | 0 |
| gen16_loops_a | Dynamic A* | solved | 27.2 s | 31.9 s | **19.7 s** | 0 |
| gen16_loops_a | Trémaux | solved | 120.0 s | 24.9 s | **19.9 s** | 0 |
| gen16_loops_a | Wall Follower | fails: ERROR left-hand rule is looping: the goal is ... | - | - | - | 0 |
| gen16_loops_b | Flood-Fill | solved | 15.0 s | 16.8 s | **12.1 s** | 0 |
| gen16_loops_b | Turn-Weighted Dijkstra | solved | 15.0 s | 16.8 s | **12.1 s** | 0 |
| gen16_loops_b | Dynamic A* | solved | 15.0 s | 16.8 s | **12.1 s** | 0 |
| gen16_loops_b | Trémaux | solved | 15.0 s | 16.8 s | **12.1 s** | 0 |
| gen16_loops_b | Wall Follower | fails: ERROR left-hand rule is looping: the goal is ... | - | - | - | 0 |
| gen16_perfect_a | Flood-Fill | solved | 72.9 s | 73.4 s | **60.1 s** | 0 |
| gen16_perfect_a | Turn-Weighted Dijkstra | solved | 72.9 s | 73.4 s | **60.1 s** | 0 |
| gen16_perfect_a | Dynamic A* | solved | 72.9 s | 73.4 s | **60.1 s** | 0 |
| gen16_perfect_a | Trémaux | solved | 81.7 s | 73.5 s | **59.8 s** | 0 |
| gen16_perfect_a | Wall Follower | solved | 86.6 s | 73.7 s | **60.0 s** | 0 |
| gen16_perfect_b | Flood-Fill | solved | 34.6 s | 36.3 s | **29.2 s** | 0 |
| gen16_perfect_b | Turn-Weighted Dijkstra | solved | 34.6 s | 36.3 s | **29.2 s** | 0 |
| gen16_perfect_b | Dynamic A* | solved | 34.6 s | 36.3 s | **29.2 s** | 0 |
| gen16_perfect_b | Trémaux | solved | 37.5 s | 35.5 s | **29.0 s** | 0 |
| gen16_perfect_b | Wall Follower | solved | 37.5 s | 35.5 s | **29.0 s** | 0 |
| uk1987f | Flood-Fill | solved | 42.2 s | 34.0 s | **20.7 s** | 0 |
| uk1987f | Turn-Weighted Dijkstra | solved | 42.2 s | 40.1 s | **20.7 s** | 0 |
| uk1987f | Dynamic A* | solved | 42.2 s | 40.1 s | **20.7 s** | 0 |
| uk1987f | Trémaux | solved | 182.9 s | 25.2 s | **20.5 s** | 0 |
| uk1987f | Wall Follower | fails: ERROR left-hand rule is looping: the goal is ... | - | - | - | 0 |

## Reading the table

- Flood-Fill, Dijkstra and A* find the same speed-run route (within 0.4 s). What differs is
  which of several equal-cost routes they take on the way back; neither planner wins everywhere
  (see decision D15 in `decisions.md`).
- Trémaux explores far more than it needs on mazes with loops (UK 1987: 182.9 s against 42.2 s)
  but never fails to find the goal.
- The Wall Follower solves the `perfect` mazes (about as fast as the others there) and stops with
  an error after one loop on every maze whose goal is a free-standing island. That is the
  expected behaviour of the algorithm, reported instead of driving in circles.
