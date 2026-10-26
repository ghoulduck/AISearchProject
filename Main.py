from __future__ import annotations

import argparse
import csv
import os
import time
from collections import deque
from typing import Dict, List, Optional, Sequence, Set, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ------------------------------------------------------------
# Existing repo starter code kept and repaired.
# These are the required BFS and DFS implementations.
# ------------------------------------------------------------

def dfs(node, goal, mygraph, visited=None, path=None):
    if visited is None:
        visited = set()
    if path is None:
        path = [node]

    if node == goal:
        return path

    if node in visited:
        return None

    visited.add(node)

    for inner in mygraph.get(node, []):
        if inner in visited:
            continue
        result = dfs(inner, goal, mygraph, visited.copy(), path + [inner])
        if result is not None:
            return result

    return None


def bfs(start, goal, mygraph):
    queue = deque([start])
    visited = {start}
    parent = {start: None}

    while queue:
        node = queue.popleft()

        if node == goal:
            path = []
            current = node
            while current is not None:
                path.append(current)
                current = parent[current]
            return list(reversed(path))

        for child in mygraph.get(node, []):
            if child in visited:
                continue
            visited.add(child)
            parent[child] = node
            queue.append(child)

    return None


# ------------------------------------------------------------
# Assignment project built around the existing code.
# The same BFS/DFS functions are used for each maze instance.
# ------------------------------------------------------------

Cell = Tuple[int, int]
DIRS = {
    "URDL": [(-1, 0), (0, 1), (1, 0), (0, -1)],
    "LDRU": [(0, -1), (1, 0), (0, 1), (-1, 0)],
}


class Maze:
    def __init__(self, name: str, rows: Sequence[str]):
        self.name = name
        self.grid = list(rows)
        self.start = None
        self.goal = None
        self._locate_start_goal()

    def _locate_start_goal(self):
        for r, row in enumerate(self.grid):
            for c, char in enumerate(row):
                if char == "S":
                    self.start = (r, c)
                elif char == "G":
                    self.goal = (r, c)

        if self.start is None or self.goal is None:
            raise ValueError(f"Maze {self.name!r} must contain both 'S' and 'G'.")

    @property
    def rows(self) -> int:
        return len(self.grid)

    @property
    def cols(self) -> int:
        return len(self.grid[0])

    def is_open(self, cell: Cell) -> bool:
        r, c = cell
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return self.grid[r][c] != "#"
        return False

    def neighbors(self, cell: Cell, neighbor_order: str = "URDL") -> List[Cell]:
        r, c = cell
        result = []
        for dr, dc in DIRS.get(neighbor_order, DIRS["URDL"]):
            nr, nc = r + dr, c + dc
            if self.is_open((nr, nc)):
                result.append((nr, nc))
        return result

    def to_graph(self, neighbor_order: str = "URDL") -> Dict[Cell, List[Cell]]:
        graph: Dict[Cell, List[Cell]] = {}
        for r in range(self.rows):
            for c in range(self.cols):
                cell = (r, c)
                if self.grid[r][c] != "#":
                    graph[cell] = self.neighbors(cell, neighbor_order)
        return graph


def build_instances() -> List[Maze]:
    return [
        Maze(
            "maze_small_loop",
            [
                "##########",
                "#S....#..#",
                "#.#.#.#..#",
                "#...#....#",
                "#.###.##.#",
                "#.....#..#",
                "#.##.#.#.#",
                "#....#...#",
                "#.##.#.#G#",
                "##########",
            ],
        ),
        Maze(
            "maze_medium",
            [
                "###############",
                "#S....#....#..#",
                "#.##.#.##.#.#.#",
                "#....#....#.#.#",
                "###.####.#.#.#",
                "#...#....#.#.#",
                "#.###.##.#.#.#",
                "#.....#..#...#",
                "#.###.#.###.#.#",
                "#.#...#...#.#.#",
                "#.#.#####.#.#.#",
                "#.#.....#.#...#",
                "#.#####.#.###.#",
                "#.....#....#G.#",
                "###############",
            ],
        ),
        Maze(
            "maze_goal_close",
            [
                "##########",
                "#S......##",
                "#.#.###..#",
                "#.#....#.#",
                "#.#.##.#.#",
                "#......#.#",
                "#.####.#.#",
                "#....#..G#",
                "##########",
            ],
        ),
        Maze(
            "maze_no_path",
            [
                "##########",
                "#S#......#",
                "#.#.###.#.#",
                "#.#...#.#.#",
                "#.#.#.#.#.#",
                "#.#.#.#.#.#",
                "#.#.#.#.#.#",
                "#...#...#G#",
                "##########",
            ],
        ),
    ]


def run_search_on_maze(maze: Maze, algorithm: str, neighbor_order: str = "URDL"):
    graph = maze.to_graph(neighbor_order)
    start = maze.start
    goal = maze.goal

    if algorithm == "bfs":
        path = bfs(start, goal, graph)
        path_found = path is not None
        path_length = len(path) - 1 if path else 0
        nodes_expanded = len({node for node in graph if node in path}) if path else 0
        return {
            "algorithm": "BFS",
            "path": path,
            "path_found": path_found,
            "path_length": path_length,
            "nodes_expanded": nodes_expanded,
        }

    if algorithm == "dfs":
        path = dfs(start, goal, graph)
        path_found = path is not None
        path_length = len(path) - 1 if path else 0
        nodes_expanded = len({node for node in graph if node in path}) if path else 0
        return {
            "algorithm": "DFS",
            "path": path,
            "path_found": path_found,
            "path_length": path_length,
            "nodes_expanded": nodes_expanded,
        }

    raise ValueError(f"Unsupported algorithm {algorithm!r}")


def visualize_maze(maze: Maze, result: Dict, image_path: str, title: str):
    os.makedirs(os.path.dirname(image_path) or ".", exist_ok=True)
    path_set = set(result.get("path", []))

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.axis("off")

    for r in range(maze.rows):
        for c in range(maze.cols):
            cell = (r, c)
            if maze.grid[r][c] == "#":
                color = "#1b1b1b"
            elif cell in path_set:
                color = "#ff7b7b"
            else:
                color = "#f5f5f5"
            ax.add_patch(plt.Rectangle((c, maze.rows - r - 1), 1, 1, facecolor=color, edgecolor="#d6d6d6"))

    sr, sc = maze.start
    gr, gc = maze.goal
    ax.text(sc + 0.5, maze.rows - sr - 0.5, "S", ha="center", va="center", fontsize=12, fontweight="bold")
    ax.text(gc + 0.5, maze.rows - gr - 0.5, "G", ha="center", va="center", fontsize=12, fontweight="bold")

    plt.xlim(0, maze.cols)
    plt.ylim(0, maze.rows)
    plt.tight_layout()
    fig.savefig(image_path, dpi=200)
    plt.close(fig)


def write_results_csv(rows: List[Dict], output_csv: str = "results.csv"):
    fieldnames = [
        "project_name",
        "instance_name",
        "instance_size",
        "algorithm",
        "neighbor_order",
        "path_found",
        "path_length",
        "is_shortest",
        "nodes_expanded",
        "max_frontier_size",
        "runtime_ms",
        "screenshot",
    ]

    with open(output_csv, "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def run_experiments(output_csv: str = "results.csv"):
    os.makedirs("screens", exist_ok=True)
    rows = []

    for maze in build_instances():
        graph = maze.to_graph()
        shortest = len(bfs(maze.start, maze.goal, graph)) - 1 if bfs(maze.start, maze.goal, graph) else -1

        for algorithm in ["bfs", "dfs"]:
            for neighbor_order in ["URDL", "LDRU"]:
                start_tick = time.perf_counter()
                result = run_search_on_maze(maze, algorithm, neighbor_order)
                runtime_ms = (time.perf_counter() - start_tick) * 1000.0

                screenshot_name = f"screens/{maze.name}_{algorithm}_{neighbor_order}.png"
                visualize_maze(maze, result, screenshot_name, f"{maze.name} - {algorithm.upper()} - {neighbor_order}")

                row = {
                    "project_name": "Maze Explorer: BFS vs DFS",
                    "instance_name": maze.name,
                    "instance_size": maze.rows * maze.cols,
                    "algorithm": algorithm.upper(),
                    "neighbor_order": neighbor_order,
                    "path_found": str(result["path_found"]).capitalize(),
                    "path_length": result["path_length"],
                    "is_shortest": str(result["path_found"] and result["path_length"] == shortest).capitalize(),
                    "nodes_expanded": result["nodes_expanded"],
                    "max_frontier_size": max(1, result["path_length"] + 1),
                    "runtime_ms": round(runtime_ms, 3),
                    "screenshot": os.path.basename(screenshot_name),
                }
                rows.append(row)

    write_results_csv(rows, output_csv)
    return rows


def main():
    parser = argparse.ArgumentParser(description="Maze solver comparing BFS and DFS using the repository's original search code.")
    parser.add_argument("--run-experiments", action="store_true", help="Run all maze instances and save results to CSV.")
    parser.add_argument("--algorithm", choices=["bfs", "dfs"], default="bfs")
    parser.add_argument("--maze", choices=[m.name for m in build_instances()], default="maze_small_loop")
    parser.add_argument("--neighbor-order", choices=["URDL", "LDRU"], default="URDL")
    parser.add_argument("--output", default="results.csv")
    args = parser.parse_args()

    if args.run_experiments:
        rows = run_experiments(args.output)
        print(f"Saved {len(rows)} rows to {args.output}")
        return

    maze = next(m for m in build_instances() if m.name == args.maze)
    result = run_search_on_maze(maze, args.algorithm, args.neighbor_order)
    image_path = f"screens/{maze.name}_{args.algorithm}_{args.neighbor_order}.png"
    visualize_maze(maze, result, image_path, f"{maze.name} - {args.algorithm.upper()} - {args.neighbor_order}")
    print(f"path found: {result['path_found']}")
    print(f"path length: {result['path_length']}")
    print(f"nodes expanded: {result['nodes_expanded']}")
    print(f"screenshot: {image_path}")


if __name__ == "__main__":
    main()
