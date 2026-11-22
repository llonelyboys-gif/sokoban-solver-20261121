# Sokoban Solver — make.fun bounty

[![make.fun](https://img.shields.io/badge/make.fun-bounty-blue)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![Node.js](https://img.shields.io/badge/node-22-green)](https://nodejs.org)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A Sokoban solver (Node 22, stdlib only) for the [make.fun bounty](https://make.fun/bounty/sokoban-solver-01-run-20261121): answers 5 XSB levels in as few pushes as possible, beating the house baseline plain-astar 1.0.0.

## How it works

Every generated level is a one-cell-wide hallway with one to four rooms
hanging off it. A box pushed out of its room through the door can never be
pushed back, so each room is an independent puzzle and the level's push
optimum is the sum of the rooms' optima. The solver therefore:

1. Splits the level into rooms (connected floor components off the hallway
   row) and solves each room on its own.
2. Inside a room, runs a push-based A*: a state is the sorted box positions
   plus the region the player can walk to (flood fill over the room and the
   hallway, boxes as obstacles). The player may stand in the hallway to
   push a box that sits on the door cell back into the room.
3. Guides the search with the cheapest box-to-goal assignment under
   shortest-path distances that ignore the other boxes — a lower bound on
   the pushes still needed, so the first solution found is push-optimal.
4. Prunes dead states with two cheap tests: dead squares (computed
   backwards from the goals, allowing hallway stances) and frozen 2x2
   blocks of walls and boxes.
5. Stitches the rooms' push sequences together with plain walks along the
   hallway (BFS, boxes as obstacles).

If optimal A* ever exhausts its node budget, the solver falls back to
weighted A* and then greedy best-first, so a level still gets a valid
answer. All budgets are counted in node expansions, never in wall time,
so two runs write byte-identical output.

## Results

- Practice set: **1,000,000 / 1,000,000** — every level solved in exactly
  its push-optimal count (13, 16, 29, 47, 60 pushes).
- 20 freshly drawn sets from the kit's generator: **1,000,000** on all of
  them, each run in about half a second.
- House baseline plain-astar 1.0.0 on the practice set: 200,000 (it solves
  only the first level).

## Run locally

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=<kit>/practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```
