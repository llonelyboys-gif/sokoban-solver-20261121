// SPDX-License-Identifier: MIT
//
// Sokoban solver for make.fun's sokoban-solver-01-run-20261121.
//
// Idea
// ----
// Every generated level is a one-cell-wide hallway with one to four rooms
// hanging off it. A box pushed out of its room through the door can never be
// pushed back, so each room is an independent puzzle: the boxes of a room
// stay in that room and its goals are all inside it. The level's push
// optimum is the sum of the rooms' optima, so this program solves each room
// on its own and stitches the push sequences together with plain walks
// along the hallway.
//
// Inside one room the search is a push-based A*: a state is the sorted box
// positions plus the region the player can walk to (a flood fill from the
// player's cell over the room and the hallway, boxes as obstacles). A move
// is one push. The heuristic is the cheapest assignment of boxes to goals
// under shortest-path distances that ignore the other boxes, which never
// overestimates the pushes still needed. Dead states are cut early with two
// cheap tests: a box standing on a square from which no goal can ever be
// reached by pushes (computed backwards from the goals, allowing the player
// to stand in the hallway), and a frozen 2x2 block of walls and boxes.
// The search is budgeted in node expansions, never in wall time, so two
// runs always write the same bytes.
//
// Only Node's built-ins are used. No clock, no randomness, no network.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT_DIR = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT_DIR = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Expansion budgets. Counted in nodes, never in wall time, so the program
// is deterministic. Tuned after measuring 20 freshly drawn sets (plus the
// practice set): the heaviest whole set needed 8,045 expansions, so these
// caps sit 15-35x above anything observed, while the global cap bounds the
// pathological case: even at the slowest measured rate (~20k expansions/s,
// uniform-cost search with a degenerate heuristic) 300,000 expansions cost
// ~15 s, far inside the 60 s wall budget. Without the global cap a room
// that missed on every pass could burn the whole wall budget and score
// nothing at all; with it, only that level is lost.
const ASTAR_BUDGET = 120000;
const WEIGHTED_BUDGET = 150000;
const GREEDY_BUDGET = 150000;
const GLOBAL_BUDGET = 300000;

let globalExpansions = 0;

const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

// ---------------------------------------------------------------------------
// Level parsing
// ---------------------------------------------------------------------------

function parseLevel(text) {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const box = new Uint8Array(w * h);
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') box[i] = 1;
      if (ch === '@' || ch === '+') player = i;
    }
  });
  return { w, h, wall, goal, box, player };
}

// Split the level into rooms: connected components of floor cells that are
// not in the hallway row (the row the player starts in). Each component
// opens onto the hallway through exactly one door cell.
function findRooms(level) {
  const { w, h, wall, player } = level;
  const hallRow = (player / w) | 0;
  const seen = new Uint8Array(w * h);
  const rooms = [];
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const i = y * w + x;
      if (wall[i] || y === hallRow || seen[i]) continue;
      const cells = [];
      const stack = [i];
      seen[i] = 1;
      while (stack.length > 0) {
        const c = stack.pop();
        cells.push(c);
        const cx = c % w;
        const cy = (c / w) | 0;
        for (const d of DIRS) {
          const nx = cx + d.dx;
          const ny = cy + d.dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
          const n = ny * w + nx;
          if (wall[n] || ny === hallRow || seen[n]) continue;
          seen[n] = 1;
          stack.push(n);
        }
      }
      // The door is the room cell touching a hallway floor cell.
      let door = -1;
      let doorHall = -1;
      for (const c of cells) {
        const cx = c % w;
        const cy = (c / w) | 0;
        for (const d of DIRS) {
          const nx = cx + d.dx;
          const ny = cy + d.dy;
          if (ny !== hallRow || nx < 0 || nx >= w) continue;
          const n = ny * w + nx;
          if (!wall[n]) {
            door = c;
            doorHall = n;
            break;
          }
        }
        if (door >= 0) break;
      }
      rooms.push({ cells, door, doorHall });
    }
  }
  return { rooms, hallRow };
}

// ---------------------------------------------------------------------------
// Small binary heap ordered by (f, h, insertion order) for determinism.
// ---------------------------------------------------------------------------

function heapLess(a, b) {
  if (a.f !== b.f) return a.f < b.f;
  if (a.h !== b.h) return a.h < b.h;
  return a.seq < b.seq;
}

function heapPush(heap, item) {
  heap.push(item);
  let i = heap.length - 1;
  while (i > 0) {
    const p = (i - 1) >> 1;
    if (!heapLess(heap[i], heap[p])) break;
    const t = heap[i];
    heap[i] = heap[p];
    heap[p] = t;
    i = p;
  }
}

function heapPop(heap) {
  const top = heap[0];
  const last = heap.pop();
  if (heap.length > 0) {
    heap[0] = last;
    let i = 0;
    for (;;) {
      const l = i * 2 + 1;
      const r = l + 1;
      let m = i;
      if (l < heap.length && heapLess(heap[l], heap[m])) m = l;
      if (r < heap.length && heapLess(heap[r], heap[m])) m = r;
      if (m === i) break;
      const t = heap[i];
      heap[i] = heap[m];
      heap[m] = t;
      i = m;
    }
  }
  return top;
}

const POPCOUNT = [0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4];

// ---------------------------------------------------------------------------
// One room's solver context: everything the search needs, precomputed once.
// ---------------------------------------------------------------------------

function buildRoom(level, room, hallCells) {
  const { w, h, wall, goal, box } = level;
  const cells = room.cells;
  const n = cells.length;
  const localOf = new Int32Array(w * h).fill(-1);
  cells.forEach((g, k) => {
    localOf[g] = k;
  });
  const gx = new Int32Array(n);
  const gy = new Int32Array(n);
  cells.forEach((g, k) => {
    gx[k] = g % w;
    gy[k] = (g / w) | 0;
  });

  // Room adjacency: neighbour room cell per direction, -1 when there is none.
  const adj = new Array(n);
  for (let k = 0; k < n; k++) {
    const row = [-1, -1, -1, -1];
    for (let d = 0; d < 4; d++) {
      const nx = gx[k] + DIRS[d].dx;
      const ny = gy[k] + DIRS[d].dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      row[d] = localOf[ny * w + nx];
    }
    adj[k] = row;
  }

  // Walk graph: the room's cells plus the hallway's floor cells, compactly
  // numbered. The player may walk anywhere in it (boxes are the only
  // obstacles), including standing in the hallway to push a box that sits
  // on the door cell back into the room.
  const walkIdx = new Int32Array(w * h).fill(-1);
  const walkCells = [];
  for (const g of cells) {
    walkIdx[g] = walkCells.length;
    walkCells.push(g);
  }
  for (const g of hallCells) {
    if (walkIdx[g] < 0) {
      walkIdx[g] = walkCells.length;
      walkCells.push(g);
    }
  }
  const wn = walkCells.length;
  const walkNb = new Array(wn);
  for (let k = 0; k < wn; k++) {
    const g = walkCells[k];
    const x = g % w;
    const y = (g / w) | 0;
    const row = [-1, -1, -1, -1];
    for (let d = 0; d < 4; d++) {
      const nx = x + DIRS[d].dx;
      const ny = y + DIRS[d].dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      row[d] = walkIdx[ny * w + nx];
    }
    walkNb[k] = row;
  }
  const BIT = new Array(wn);
  for (let k = 0; k < wn; k++) BIT[k] = 1n << BigInt(k);
  let hallMask = 0n;
  for (const g of hallCells) hallMask |= BIT[walkIdx[g]];

  const goalLocals = [];
  const boxLocals = [];
  for (let k = 0; k < n; k++) {
    if (goal[cells[k]]) goalLocals.push(k);
    if (box[cells[k]]) boxLocals.push(k);
  }
  boxLocals.sort((a, b) => a - b);
  const goalSet = new Uint8Array(n);
  for (const g of goalLocals) goalSet[g] = 1;

  // All-pairs push distances over room cells (ignoring other boxes).
  const dist = new Int16Array(n * n).fill(-1);
  for (let s = 0; s < n; s++) {
    const q = [s];
    dist[s * n + s] = 0;
    for (let qi = 0; qi < q.length; qi++) {
      const c = q[qi];
      for (const nb of adj[c]) {
        if (nb < 0 || dist[s * n + nb] >= 0) continue;
        dist[s * n + nb] = dist[s * n + c] + 1;
        q.push(nb);
      }
    }
  }

  // Live cells: a cell is live when a box standing there can still be pushed
  // onto a goal, ignoring the other boxes. Computed backwards from the goals:
  // a box at x can be pushed in direction d onto a live cell y when the
  // stance cell x - d is somewhere the player can stand (a room cell or a
  // hallway cell). Anything else is a dead square.
  const live = new Uint8Array(n);
  const liveStack = [];
  for (const g of goalLocals) {
    live[g] = 1;
    liveStack.push(g);
  }
  while (liveStack.length > 0) {
    const y = liveStack.pop();
    for (let d = 0; d < 4; d++) {
      const x = adj[y][d ^ 1];
      if (x < 0 || live[x]) continue;
      const sx = gx[x] + DIRS[d ^ 1].dx;
      const sy = gy[x] + DIRS[d ^ 1].dy;
      let stanceOk = adj[x][d ^ 1] >= 0;
      if (!stanceOk && sx >= 0 && sy >= 0 && sx < w && sy < h) {
        stanceOk = walkIdx[sy * w + sx] >= 0;
      }
      if (!stanceOk) continue;
      live[x] = 1;
      liveStack.push(x);
    }
  }

  // 2x2 blocks, each as the room cells it holds. A block whose room cells are
  // all occupied by boxes, with at least one of them off its goal, freezes
  // those boxes unless one of them can still be pushed out of the block.
  const blocks = [];
  for (let y = 0; y < h - 1; y++) {
    for (let x = 0; x < w - 1; x++) {
      const members = [];
      for (const [dx, dy] of [
        [0, 0],
        [1, 0],
        [0, 1],
        [1, 1],
      ]) {
        const li = localOf[(y + dy) * w + (x + dx)];
        if (li >= 0) members.push(li);
      }
      if (members.length > 0) blocks.push(members);
    }
  }

  return {
    w,
    h,
    cells,
    n,
    localOf,
    gx,
    gy,
    adj,
    walkIdx,
    walkCells,
    wn,
    walkNb,
    BIT,
    hallMask,
    goalLocals,
    boxLocals,
    goalSet,
    dist,
    live,
    blocks,
    doorHall: room.doorHall,
  };
}

// ---------------------------------------------------------------------------
// The search itself.
// ---------------------------------------------------------------------------

function solveRoom(level, room, hallCells, needExit) {
  const ctx = buildRoom(level, room, hallCells);
  const m = ctx.boxLocals.length;
  if (m === 0) return [];
  if (m !== ctx.goalLocals.length) return null;

  const { n, adj, walkNb, walkIdx, BIT, hallMask, goalSet, dist, live, blocks } = ctx;
  const goalLocals = ctx.goalLocals;

  // Flood fill from a walk cell, boxes marked in boxMark. Returns the region
  // as a bitmask over the walk graph.
  const visited = new Int32Array(ctx.wn);
  let stamp = 0;
  const boxMark = new Uint8Array(ctx.wn);
  const flood = (start) => {
    stamp++;
    let mask = 0n;
    const stack = [start];
    visited[start] = stamp;
    while (stack.length > 0) {
      const c = stack.pop();
      mask |= BIT[c];
      for (const nb of walkNb[c]) {
        if (nb < 0 || boxMark[nb] || visited[nb] === stamp) continue;
        visited[nb] = stamp;
        stack.push(nb);
      }
    }
    return mask;
  };

  // Cheapest box -> goal assignment (subset DP), a lower bound on the pushes
  // still needed. Infinity when some box cannot reach any goal.
  const heuristic = (boxes) => {
    for (const b of boxes) if (!live[b]) return Infinity;
    const N = 1 << m;
    const dp = new Float64Array(N).fill(Infinity);
    dp[0] = 0;
    for (let mask = 0; mask < N; mask++) {
      const cur = dp[mask];
      if (cur === Infinity) continue;
      const i = POPCOUNT[mask];
      if (i >= m) continue;
      for (let j = 0; j < m; j++) {
        if (mask & (1 << j)) continue;
        const d = dist[boxes[i] * n + goalLocals[j]];
        if (d < 0) continue;
        const nm = mask | (1 << j);
        const v = cur + d;
        if (v < dp[nm]) dp[nm] = v;
      }
    }
    return dp[N - 1];
  };

  // True when a 2x2 block of walls and boxes freezes an off-goal box.
  const frozen = (boxes, occ) => {
    for (const members of blocks) {
      let allOcc = true;
      let offGoal = false;
      for (const cell of members) {
        if (!occ[cell]) {
          allOcc = false;
          break;
        }
        if (!goalSet[cell]) offGoal = true;
      }
      if (!allOcc || !offGoal) continue;
      let canMove = false;
      for (const cell of members) {
        for (let d = 0; d < 4; d++) {
          const t = adj[cell][d];
          if (t < 0 || occ[t]) continue;
          const sx = ctx.gx[cell] + DIRS[d ^ 1].dx;
          const sy = ctx.gy[cell] + DIRS[d ^ 1].dy;
          if (sx < 0 || sy < 0 || sx >= ctx.w || sy >= ctx.h) continue;
          const sLocal = ctx.localOf[sy * ctx.w + sx];
          if (sLocal >= 0) {
            if (!occ[sLocal]) {
              canMove = true;
              break;
            }
          } else if (walkIdx[sy * ctx.w + sx] >= 0) {
            canMove = true;
            break;
          }
        }
        if (canMove) break;
      }
      if (!canMove) return true;
    }
    return false;
  };

  const isGoal = (boxes, region) => {
    for (const b of boxes) if (!goalSet[b]) return false;
    if (needExit && (region & hallMask) === 0n) return false;
    return true;
  };

  const search = (weight, budget, greedy) => {
    const nodes = [];
    const heap = [];
    const best = new Map();
    let seq = 0;

    const startBoxes = [...ctx.boxLocals];
    const startPlayer = walkIdx[ctx.doorHall];
    const occ0 = new Uint8Array(n);
    for (const b of startBoxes) {
      occ0[b] = 1;
      boxMark[walkIdx[ctx.cells[b]]] = 1;
    }
    const startRegion = flood(startPlayer);
    for (const b of startBoxes) boxMark[walkIdx[ctx.cells[b]]] = 0;
    if (frozen(startBoxes, occ0)) return null;
    const h0 = heuristic(startBoxes);
    if (h0 === Infinity) return null;
    const f0 = greedy ? h0 : h0 * weight;
    const startKey = startBoxes.join(',') + '|' + startRegion.toString();
    best.set(startKey, 0);
    nodes.push({
      idx: 0,
      boxes: startBoxes,
      g: 0,
      h: h0,
      f: f0,
      seq: seq++,
      parent: -1,
      pushBox: -1,
      pushDir: -1,
      player: startPlayer,
      region: startRegion,
    });
    heapPush(heap, nodes[0]);

    let expansions = 0;
    while (heap.length > 0) {
      const node = heapPop(heap);
      const key = node.boxes.join(',') + '|' + node.region.toString();
      if (!greedy) {
        const known = best.get(key);
        if (known !== undefined && node.g > known) continue;
      } else if (node.parent !== -1 && best.has(key) && best.get(key) < node.g) {
        continue;
      }
      if (isGoal(node.boxes, node.region)) {
        const pushes = [];
        let cur = node.idx;
        while (nodes[cur].parent !== -1) {
          pushes.push({ box: nodes[cur].pushBox, dir: nodes[cur].pushDir });
          cur = nodes[cur].parent;
        }
        pushes.reverse();
        return pushes;
      }
      if (expansions >= budget || globalExpansions >= GLOBAL_BUDGET) return null;
      expansions++;
      globalExpansions++;

      const occ = new Uint8Array(n);
      for (const b of node.boxes) {
        occ[b] = 1;
        boxMark[walkIdx[ctx.cells[b]]] = 1;
      }
      for (let bi = 0; bi < node.boxes.length; bi++) {
        const b = node.boxes[bi];
        for (let d = 0; d < 4; d++) {
          const t = adj[b][d];
          if (t < 0 || occ[t]) continue;
          const sx = ctx.gx[b] + DIRS[d ^ 1].dx;
          const sy = ctx.gy[b] + DIRS[d ^ 1].dy;
          if (sx < 0 || sy < 0 || sx >= ctx.w || sy >= ctx.h) continue;
          const stance = walkIdx[sy * ctx.w + sx];
          if (stance < 0 || (node.region & BIT[stance]) === 0n) continue;
          // Apply the push: box b -> t, player ends up on b's old cell.
          const childBoxes = [...node.boxes];
          childBoxes[bi] = t;
          childBoxes.sort((a, b2) => a - b2);
          const childH = heuristic(childBoxes);
          if (childH === Infinity) continue;
          if (frozen(childBoxes, occWithSwap(occ, b, t))) continue;
          occ[b] = 0;
          occ[t] = 1;
          boxMark[walkIdx[ctx.cells[b]]] = 0;
          boxMark[walkIdx[ctx.cells[t]]] = 1;
          const childRegion = flood(walkIdx[ctx.cells[b]]);
          boxMark[walkIdx[ctx.cells[b]]] = 1;
          boxMark[walkIdx[ctx.cells[t]]] = 0;
          occ[b] = 1;
          occ[t] = 0;
          const g = node.g + 1;
          const childKey = childBoxes.join(',') + '|' + childRegion.toString();
          if (greedy) {
            if (best.has(childKey)) continue;
            best.set(childKey, g);
          } else {
            const known = best.get(childKey);
            if (known !== undefined && known <= g) continue;
            best.set(childKey, g);
          }
          const f = greedy ? childH : g + weight * childH;
          const child = {
            idx: nodes.length,
            boxes: childBoxes,
            g,
            h: childH,
            f,
            seq: seq++,
            parent: node.idx,
            pushBox: b,
            pushDir: d,
            player: walkIdx[ctx.cells[b]],
            region: childRegion,
          };
          nodes.push(child);
          heapPush(heap, child);
        }
      }
      for (const b of node.boxes) boxMark[walkIdx[ctx.cells[b]]] = 0;
    }
    return null;
  };

  // A frozen check needs the occupancy with one box swapped; build it cheaply.
  function occWithSwap(occ, from, to) {
    const copy = new Uint8Array(occ);
    copy[from] = 0;
    copy[to] = 1;
    return copy;
  }

  const passes = [
    { weight: 1, budget: ASTAR_BUDGET, greedy: false },
    { weight: 2, budget: WEIGHTED_BUDGET, greedy: false },
    { weight: 0, budget: GREEDY_BUDGET, greedy: true },
  ];
  for (const p of passes) {
    const found = search(p.weight, p.budget, p.greedy);
    if (found) return found;
  }
  return null;
}

// ---------------------------------------------------------------------------
// Whole-level assembly: solve each room, stitch the pushes together with
// walks (plain BFS over the board, boxes as obstacles).
// ---------------------------------------------------------------------------

function bfsPath(level, from, to, boxes) {
  if (from === to) return '';
  const { w, h, wall } = level;
  const prev = new Int32Array(w * h).fill(-2);
  const prevDir = new Int8Array(w * h).fill(-1);
  const queue = [from];
  prev[from] = -1;
  for (let qi = 0; qi < queue.length; qi++) {
    const c = queue[qi];
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DIRS[d].dx;
      const ny = cy + DIRS[d].dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (wall[n] || boxes.has(n) || prev[n] !== -2) continue;
      prev[n] = c;
      prevDir[n] = d;
      if (n === to) {
        let out = '';
        let cur = n;
        while (cur !== from) {
          out = DIRS[prevDir[cur]].ch + out;
          cur = prev[cur];
        }
        return out;
      }
      queue.push(n);
    }
  }
  return null;
}

function solveLevel(text) {
  const level = parseLevel(text);
  const { w, h } = level;
  const { rooms, hallRow } = findRooms(level);
  const hallCells = [];
  for (let x = 0; x < w; x++) {
    const i = hallRow * w + x;
    if (!level.wall[i]) hallCells.push(i);
  }
  rooms.sort((a, b) => a.door - b.door);
  const boxes = new Set();
  for (let i = 0; i < w * h; i++) if (level.box[i]) boxes.add(i);
  let player = level.player;
  let moves = '';
  for (let ri = 0; ri < rooms.length; ri++) {
    const room = rooms[ri];
    if (room.door < 0) return '';
    const pushes = solveRoom(level, room, hallCells, ri < rooms.length - 1);
    if (!pushes) return '';
    for (const p of pushes) {
      const boxGlobal = room.cells[p.box];
      const d = DIRS[p.dir];
      const stance = boxGlobal - d.dy * w - d.dx;
      const walk = bfsPath(level, player, stance, boxes);
      if (walk === null) return '';
      moves += walk + d.ch;
      boxes.delete(boxGlobal);
      boxes.add(boxGlobal + d.dy * w + d.dx);
      player = boxGlobal;
    }
  }
  return moves;
}

// ---------------------------------------------------------------------------
// Entry point: one solution.txt per instance id.
// ---------------------------------------------------------------------------

for (const id of readdirSync(INPUT_DIR).sort()) {
  const moves = solveLevel(readFileSync(join(INPUT_DIR, id, 'level.txt'), 'utf8'));
  mkdirSync(join(OUTPUT_DIR, id), { recursive: true });
  writeFileSync(join(OUTPUT_DIR, id, 'solution.txt'), moves + '\n');
}
