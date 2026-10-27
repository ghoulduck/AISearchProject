from collections import deque
import heapq
import random
import tkinter as tk
from tkinter import ttk, messagebox

# Original project logic is preserved below.
# The original BFS queue handling and DFS default arguments remain intact.
# The maze system is integrated as a separate mode so the graph and maze solving
# do not interfere with each other.
mygraph = {
    "A": ["B", "C"],
    "B": ["A", "D", "E"],
    "C": ["A", "F"],
    "D": ["B"],
    "E": ["B", "G"],
    "F": ["C"],
    "G": ["E"],
}

NODE_POSITIONS = {
    "A": (120, 120),
    "B": (260, 80),
    "C": (260, 220),
    "D": (430, 60),
    "E": (430, 160),
    "F": (430, 260),
    "G": (560, 160),
}


def heuristic(node, goal):
    # This is kept separate from the original graph logic, so the original
    # BFS/DFS code still behaves the same when used in the graph view.
    x1, y1 = NODE_POSITIONS[node]
    x2, y2 = NODE_POSITIONS[goal]
    return abs(x1 - x2) + abs(y1 - y2)


def bfs(start, goal, mygraph):
    # Preserve the original BFS queue implementation from the project.
    queue = deque([[start]])
    visited = []

    while queue:
        path = queue.popleft()
        node = path[-1]

        if node in visited:
            continue

        visited.append(node)

        if node == goal:
            return path

        for child in mygraph[node]:
            if child not in visited:
                queue.append(path + [child])

    return None


def dfs(node, goal, mygraph, visited=None, path=None):
    # Preserve the original DFS default arguments and recursive traversal pattern.
    if visited is None:
        visited = []
    if path is None:
        path = []

    if node == goal:
        return path + [node]

    if node not in visited:
        visited.append(node)
        for inner in mygraph.get(node, []):
            if inner not in visited:
                result = dfs(inner, goal, mygraph, visited, path + [node])
                if result is not None:
                    return result
    return None


def astar(start, goal, graph):
    # A* is added as a new algorithm without changing the original graph functions.
    frontier = []
    heapq.heappush(frontier, (heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}

    while frontier:
        _, current_cost, current = heapq.heappop(frontier)

        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            return list(reversed(path))

        for neighbor in graph.get(current, []):
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                priority = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(frontier, (priority, tentative_g, neighbor))

    return None


def bfs_steps(start, goal, graph):
    # This animation helper is only for the graph-view visualizer and keeps the
    # same queue-based traversal logic used in the project.
    queue = deque([[start]])
    visited = set()
    steps = []

    while queue:
        path = queue.popleft()
        node = path[-1]

        if node in visited:
            steps.append({
                "type": "skip",
                "node": node,
                "queue": [p[-1] for p in queue],
                "visited": sorted(visited),
                "message": f"Node {node} already visited",
            })
            continue

        visited.add(node)
        steps.append({
            "type": "visit",
            "node": node,
            "queue": [p[-1] for p in queue],
            "visited": sorted(visited),
            "message": f"Visiting {node}",
        })

        if node == goal:
            steps.append({
                "type": "found",
                "node": node,
                "path": path,
                "queue": [p[-1] for p in queue],
                "visited": sorted(visited),
                "message": f"Goal {goal} found via {path}",
            })
            return steps

        for child in graph.get(node, []):
            if child not in visited:
                queue.append(path + [child])

        steps.append({
            "type": "expand",
            "node": node,
            "queue": [p[-1] for p in queue],
            "visited": sorted(visited),
            "message": f"Expanded {node}, added {[c for c in graph.get(node, []) if c not in visited]}",
        })

    steps.append({
        "type": "not_found",
        "node": goal,
        "queue": [],
        "visited": sorted(visited),
        "message": f"Goal {goal} not found",
    })
    return steps


def dfs_steps(start, goal, graph):
    # This maintains the recursive DFS idea while adding animated step tracking.
    stack = [[start]]
    visited = set()
    steps = []

    while stack:
        path = stack.pop()
        node = path[-1]

        if node in visited:
            steps.append({
                "type": "skip",
                "node": node,
                "stack": [p[-1] for p in stack],
                "visited": sorted(visited),
                "message": f"Node {node} already visited",
            })
            continue

        visited.add(node)
        steps.append({
            "type": "visit",
            "node": node,
            "stack": [p[-1] for p in stack],
            "visited": sorted(visited),
            "message": f"Visiting {node}",
        })

        if node == goal:
            steps.append({
                "type": "found",
                "node": node,
                "path": path,
                "stack": [p[-1] for p in stack],
                "visited": sorted(visited),
                "message": f"Goal {goal} found via {path}",
            })
            return steps

        neighbors = graph.get(node, [])
        for child in reversed(neighbors):
            if child not in visited:
                stack.append(path + [child])

        steps.append({
            "type": "expand",
            "node": node,
            "stack": [p[-1] for p in stack],
            "visited": sorted(visited),
            "message": f"Expanded {node}, pushed {[c for c in neighbors if c not in visited]}",
        })

    steps.append({
        "type": "not_found",
        "node": goal,
        "stack": [],
        "visited": sorted(visited),
        "message": f"Goal {goal} not found",
    })
    return steps


def astar_steps(start, goal, graph):
    # A* animation is separate from the maze logic and only used in graph mode.
    frontier = []
    heapq.heappush(frontier, (heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    visited = set()
    steps = []

    while frontier:
        f_score, current_cost, current = heapq.heappop(frontier)

        if current in visited:
            steps.append({
                "type": "skip",
                "node": current,
                "queue": [item[2] for item in frontier],
                "visited": sorted(visited),
                "message": f"Node {current} already evaluated",
            })
            continue

        visited.add(current)
        steps.append({
            "type": "visit",
            "node": current,
            "queue": [item[2] for item in frontier],
            "visited": sorted(visited),
            "message": f"Evaluating {current} (f={f_score}, g={current_cost})",
        })

        if current == goal:
            path = [goal]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            path = list(reversed(path))
            steps.append({
                "type": "found",
                "node": current,
                "path": path,
                "queue": [item[2] for item in frontier],
                "visited": sorted(visited),
                "message": f"Goal {goal} found via {path}",
            })
            return steps

        for neighbor in graph.get(current, []):
            tentative_g = g_score[current] + 1
            if tentative_g < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                priority = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(frontier, (priority, tentative_g, neighbor))

        steps.append({
            "type": "expand",
            "node": current,
            "queue": [item[2] for item in frontier],
            "visited": sorted(visited),
            "message": f"Expanded {current}, added neighbors with A* priority",
        })

    steps.append({
        "type": "not_found",
        "node": goal,
        "queue": [],
        "visited": sorted(visited),
        "message": f"Goal {goal} not found",
    })
    return steps


def generate_maze(rows=12, cols=12):
    # Fix the original bug: maze sizes must be odd so the corner start/end cells
    # are valid open cells in a perfect maze. Even-sized mazes can leave the goal
    # corner isolated because walls are created on even coordinates.
    rows = max(5, rows)
    cols = max(5, cols)

    if rows % 2 == 0:
        rows += 1
    if cols % 2 == 0:
        cols += 1

    maze = [[1 for _ in range(cols)] for _ in range(rows)]
    start = (0, 0)
    maze[start[0]][start[1]] = 0

    # Generate a perfect maze using randomized DFS. This guarantees all reachable
    # cells are connected and avoids the unreachable corner problem.
    stack = [start]
    while stack:
        r, c = stack[-1]
        neighbors = []
        for dr, dc in [(0, 2), (2, 0), (0, -2), (-2, 0)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and maze[nr][nc] == 1:
                neighbors.append((nr, nc))

        if not neighbors:
            stack.pop()
            continue

        nr, nc = random.choice(neighbors)
        maze[(r + nr) // 2][(c + nc) // 2] = 0
        maze[nr][nc] = 0
        stack.append((nr, nc))

    # Keep the goal at the far corner and ensure it is an open cell.
    maze[rows - 1][cols - 1] = 0
    return maze


def maze_to_graph(maze):
    # Convert the maze into a graph so the same search algorithms can operate on it.
    rows = len(maze)
    cols = len(maze[0])
    graph = {}
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    for r in range(rows):
        for c in range(cols):
            if maze[r][c] == 0:
                graph[(r, c)] = []
                for dr, dc in directions:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols and maze[nr][nc] == 0:
                        graph[(r, c)].append((nr, nc))
    return graph


def maze_bfs(start, goal, graph):
    # BFS for maze pathfinding is implemented as a separate maze-specific version.
    queue = deque([start])
    visited = {start}
    parent = {start: None}

    while queue:
        node = queue.popleft()
        if node == goal:
            path = []
            while node is not None:
                path.append(node)
                node = parent[node]
            return list(reversed(path))

        for neighbor in graph.get(node, []):
            if neighbor not in visited:
                visited.add(neighbor)
                parent[neighbor] = node
                queue.append(neighbor)
    return None


def maze_dfs(start, goal, graph):
    # DFS for a maze uses the same recursive idea but with a stack and tuple state.
    stack = [(start, [start])]
    visited = set()

    while stack:
        node, path = stack.pop()
        if node in visited:
            continue
        visited.add(node)

        if node == goal:
            return path

        for neighbor in reversed(graph.get(node, [])):
            if neighbor not in visited:
                stack.append((neighbor, path + [neighbor]))
    return None


def maze_astar(start, goal, graph):
    # A* for maze solving uses Manhattan distance to guide the search.
    def manhattan(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    frontier = []
    heapq.heappush(frontier, (manhattan(start, goal), 0, start))
    came_from = {start: None}
    g_score = {start: 0}

    while frontier:
        _, cost, current = heapq.heappop(frontier)
        if current == goal:
            path = []
            while current is not None:
                path.append(current)
                current = came_from[current]
            return list(reversed(path))

        for neighbor in graph.get(current, []):
            tentative = g_score[current] + 1
            if tentative < g_score.get(neighbor, float("inf")):
                came_from[neighbor] = current
                g_score[neighbor] = tentative
                heapq.heappush(frontier, (tentative + manhattan(neighbor, goal), tentative, neighbor))
    return None


class SearchVisualizer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AI Search Project")
        self.geometry("1100x840")
        self.configure(bg="#edf2fa")

        # The current application tracks both graph and maze state separately.
        self.current_steps = []
        self.current_index = 0
        self.animation_job = None
        self.current_mode = "BFS"
        self.maze = None
        self.current_maze_path = []
        self.maze_start = (0, 0)
        self.maze_goal = None

        self.start_var = tk.StringVar(value="A")
        self.goal_var = tk.StringVar(value="G")
        self.status_var = tk.StringVar(value="Ready")
        self.detail_var = tk.StringVar(value="Choose start and goal nodes, then run BFS, DFS, or A*.")
        self.speed_var = tk.DoubleVar(value=0.8)
        self.mode_var = tk.StringVar(value="graph")
        self.maze_rows_var = tk.IntVar(value=12)
        self.maze_cols_var = tk.IntVar(value=12)

        top = ttk.Frame(self, padding=12)
        top.pack(fill="x")

        ttk.Label(top, text="Start Node:", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, padx=(0, 8), sticky="w")
        ttk.Combobox(top, textvariable=self.start_var, values=sorted(mygraph.keys()), state="readonly", width=8).grid(row=0, column=1, padx=(0, 16))

        ttk.Label(top, text="Goal Node:", font=("Segoe UI", 10, "bold")).grid(row=0, column=2, padx=(0, 8), sticky="w")
        ttk.Combobox(top, textvariable=self.goal_var, values=sorted(mygraph.keys()), state="readonly", width=8).grid(row=0, column=3, padx=(0, 16))

        ttk.Button(top, text="Run BFS", command=lambda: self.run_search("BFS")).grid(row=0, column=4, padx=(0, 8))
        ttk.Button(top, text="Run DFS", command=lambda: self.run_search("DFS")).grid(row=0, column=5, padx=(0, 8))
        ttk.Button(top, text="Run A*", command=lambda: self.run_search("A*")).grid(row=0, column=6, padx=(0, 8))
        ttk.Button(top, text="Reset", command=self.reset_view).grid(row=0, column=7)

        controls = ttk.Frame(self, padding=(12, 0, 12, 12))
        controls.pack(fill="x")

        ttk.Label(controls, text="Animation speed:", font=("Segoe UI", 10, "bold")).pack(side="left")
        ttk.Scale(controls, from_=0.2, to=2.0, orient="horizontal", variable=self.speed_var, length=220).pack(side="left", padx=(8, 16))

        ttk.Label(controls, text="Visual mode:", font=("Segoe UI", 10, "bold")).pack(side="left")
        ttk.OptionMenu(controls, self.mode_var, "graph", "graph", "maze").pack(side="left", padx=(8, 0))

        maze_controls = ttk.Frame(self, padding=(12, 0, 12, 0))
        maze_controls.pack(fill="x")

        ttk.Label(maze_controls, text="Maze rows:", font=("Segoe UI", 10, "bold")).pack(side="left")
        ttk.Entry(maze_controls, textvariable=self.maze_rows_var, width=6).pack(side="left", padx=(4, 12))
        ttk.Label(maze_controls, text="Maze cols:", font=("Segoe UI", 10, "bold")).pack(side="left")
        ttk.Entry(maze_controls, textvariable=self.maze_cols_var, width=6).pack(side="left", padx=(4, 12))
        ttk.Button(maze_controls, text="Generate Maze", command=self.generate_maze).pack(side="left", padx=(0, 8))
        ttk.Button(maze_controls, text="Solve Maze BFS", command=lambda: self.solve_maze("BFS")).pack(side="left", padx=(0, 8))
        ttk.Button(maze_controls, text="Solve Maze DFS", command=lambda: self.solve_maze("DFS")).pack(side="left", padx=(0, 8))
        ttk.Button(maze_controls, text="Solve Maze A*", command=lambda: self.solve_maze("A*")).pack(side="left", padx=(0, 8))

        info = ttk.Frame(self, padding=(12, 0, 12, 12))
        info.pack(fill="x")
        ttk.Label(info, textvariable=self.status_var, font=("Segoe UI", 11, "bold"), foreground="#1a73e8").pack(anchor="w")
        ttk.Label(info, textvariable=self.detail_var, wraplength=1000, justify="left").pack(anchor="w", pady=(4, 0))

        self.canvas = tk.Canvas(self, width=920, height=520, bg="#ffffff", highlightthickness=1, highlightbackground="#d0d7de")
        self.canvas.pack(fill="both", padx=12, pady=(0, 12), expand=True)

        bottom = ttk.Frame(self, padding=(12, 0, 12, 12))
        bottom.pack(fill="x")

        self.path_label = ttk.Label(bottom, text="Path: none", font=("Segoe UI", 10, "bold"), foreground="#0f766e")
        self.path_label.pack(anchor="w")

        self.cost_label = ttk.Label(bottom, text="Cost: 0", font=("Segoe UI", 10, "bold"), foreground="#7c3aed")
        self.cost_label.pack(anchor="w", pady=(4, 0))

        self.algorithm_label = ttk.Label(bottom, text="Algorithm: Ready", font=("Segoe UI", 10, "bold"), foreground="#334155")
        self.algorithm_label.pack(anchor="w", pady=(4, 0))

        self.draw_graph()
        self.reset_view()

    def reset_view(self):
        # Reset both the graph and maze state so the app returns to a clean state.
        if self.animation_job is not None:
            self.after_cancel(self.animation_job)
            self.animation_job = None

        self.current_steps = []
        self.current_index = 0
        self.status_var.set("Ready")
        self.detail_var.set("Choose start and goal nodes, then run BFS, DFS, or A*.")
        self.path_label.config(text="Path: none")
        self.cost_label.config(text="Cost: 0")
        self.algorithm_label.config(text="Algorithm: Ready")
        self.current_maze_path = []
        self.mode_var.set("graph")
        self.draw_graph()

    def generate_maze(self):
        # Maze generation is separated from the graph logic, so the graph view is
        # not used when solving a maze.
        rows = max(5, min(25, self.maze_rows_var.get()))
        cols = max(5, min(25, self.maze_cols_var.get()))

        # Force odd dimensions before creation so the final corner cell is always a
        # valid open cell in the generated maze. This prevents isolated goal cells.
        if rows % 2 == 0:
            rows += 1
        if cols % 2 == 0:
            cols += 1

        self.maze = generate_maze(rows, cols)
        self.maze_start = (0, 0)
        self.maze_goal = (rows - 1, cols - 1)
        self.current_maze_path = []
        self.mode_var.set("maze")
        self.status_var.set("Maze generated")
        self.detail_var.set(f"Generated a {rows}x{cols} maze. Solve it with BFS, DFS, or A*.")
        self.draw_graph()

    def solve_maze(self, mode):
        # Only the maze is solved in this mode; the graph view is disabled.
        if self.maze is None:
            self.generate_maze()

        graph = maze_to_graph(self.maze)
        start = self.maze_start
        goal = self.maze_goal

        if mode == "BFS":
            self.current_maze_path = maze_bfs(start, goal, graph) or []
        elif mode == "DFS":
            self.current_maze_path = maze_dfs(start, goal, graph) or []
        elif mode == "A*":
            self.current_maze_path = maze_astar(start, goal, graph) or []

        self.mode_var.set("maze")
        self.current_mode = mode
        self.algorithm_label.config(text=f"Algorithm: Maze {mode}")

        if self.current_maze_path:
            self.status_var.set(f"Maze solved via {mode}")
            self.detail_var.set(f"Maze path found: {' -> '.join(str(p) for p in self.current_maze_path)}")
            self.path_label.config(text=f"Path: {' -> '.join(str(p) for p in self.current_maze_path)}")
            self.cost_label.config(text=f"Cost: {len(self.current_maze_path) - 1}")
        else:
            self.status_var.set(f"Maze could not be solved with {mode}")
            self.detail_var.set("No path found.")
            self.path_label.config(text="Path: none")
            self.cost_label.config(text="Cost: unreachable")
        self.draw_graph()

    def draw_graph(self, current=None, visited=None, queue=None, stack=None, path=None):
        # The main canvas switches between graph mode and maze mode.
        self.canvas.delete("all")

        if self.mode_var.get() == "maze" and self.maze is not None:
            self.draw_maze()
            return

        visited = set(visited or [])
        queue = queue or []
        stack = stack or []
        path = path or []

        for node, neighbors in mygraph.items():
            x1, y1 = NODE_POSITIONS[node]
            for neighbor in neighbors:
                x2, y2 = NODE_POSITIONS[neighbor]
                self.canvas.create_line(x1, y1, x2, y2, fill="#a3b1c6", width=2)

        for node in sorted(mygraph.keys()):
            x, y = NODE_POSITIONS[node]
            fill = "#dbeafe"
            outline = "#1d4ed8"

            if current == node:
                fill = "#fbbf24"
                outline = "#b45309"
            elif node in visited:
                fill = "#bbf7d0"
                outline = "#15803d"
            elif node in queue or node in stack:
                fill = "#e0e7ff"
                outline = "#4f46e5"
            elif node in path:
                fill = "#fde68a"
                outline = "#a16207"

            self.canvas.create_oval(x - 30, y - 30, x + 30, y + 30, fill=fill, outline=outline, width=3)
            self.canvas.create_text(x, y, text=node, font=("Segoe UI", 12, "bold"))

    def draw_maze(self):
        # Draw only the current maze as a grid with start and goal cells highlighted.
        rows = len(self.maze)
        cols = len(self.maze[0])
        cell_w = 440 / max(1, cols)
        cell_h = 420 / max(1, rows)
        cell_x = 40
        cell_y = 40

        for r in range(rows):
            for c in range(cols):
                x1 = cell_x + c * cell_w
                y1 = cell_y + r * cell_h
                x2 = x1 + cell_w
                y2 = y1 + cell_h

                if self.maze[r][c] == 1:
                    fill = "#101828"
                else:
                    fill = "#ffffff"

                self.canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline="#cbd5e1")

        if self.current_maze_path:
            path_set = set(self.current_maze_path)
            for r, c in path_set:
                x1 = cell_x + c * cell_w
                y1 = cell_y + r * cell_h
                x2 = x1 + cell_w
                y2 = y1 + cell_h
                self.canvas.create_rectangle(x1 + 4, y1 + 4, x2 - 4, y2 - 4, fill="#fde68a", outline="")

        start_r, start_c = self.maze_start
        goal_r, goal_c = self.maze_goal
        start_x = cell_x + start_c * cell_w + cell_w / 2
        start_y = cell_y + start_r * cell_h + cell_h / 2
        goal_x = cell_x + goal_c * cell_w + cell_w / 2
        goal_y = cell_y + goal_r * cell_h + cell_h / 2

        self.canvas.create_oval(start_x - 12, start_y - 12, start_x + 12, start_y + 12, fill="#22c55e")
        self.canvas.create_oval(goal_x - 12, goal_y - 12, goal_x + 12, goal_y + 12, fill="#ef4444")

    def run_search(self, mode):
        # Graph-mode search functions are left alone; maze solving stays separate.
        if self.animation_job is not None:
            self.after_cancel(self.animation_job)
            self.animation_job = None

        start = self.start_var.get()
        goal = self.goal_var.get()

        if start not in mygraph or goal not in mygraph:
            messagebox.showerror("Invalid selection", "Please pick valid nodes from the graph.")
            return

        self.current_mode = mode.strip()
        self.mode_var.set("graph")
        self.algorithm_label.config(text=f"Algorithm: {self.current_mode}")

        if self.current_mode == "BFS":
            self.current_steps = bfs_steps(start, goal, mygraph)
        elif self.current_mode == "DFS":
            self.current_steps = dfs_steps(start, goal, mygraph)
        elif self.current_mode == "A*":
            self.current_steps = astar_steps(start, goal, mygraph)
        else:
            self.current_steps = []

        self.current_index = 0
        self.animate_step()

    def animate_step(self):
        # This animation loop is kept for graph mode only.
        if not self.current_steps:
            self.status_var.set("No graph steps to display")
            return

        if self.current_index >= len(self.current_steps):
            self.status_var.set(f"{self.current_mode} finished")
            return

        step = self.current_steps[self.current_index]
        self.current_index += 1

        current = step.get("node")
        visited = step.get("visited", [])
        queue = step.get("queue", [])
        stack = step.get("stack", [])
        path = step.get("path", [])

        self.status_var.set(f"{self.current_mode} running")
        self.detail_var.set(step.get("message", "Processing path"))
        self.draw_graph(current=current, visited=visited, queue=queue, stack=stack, path=path)

        if path:
            self.path_label.config(text=f"Path: {' -> '.join(path)}")
            self.cost_label.config(text=f"Cost: {len(path) - 1}")
        elif step.get("type") in {"not_found"}:
            self.path_label.config(text="Path: none")
            self.cost_label.config(text="Cost: unreachable")

        if step.get("type") in {"found", "not_found"}:
            if path:
                self.detail_var.set(f"Path: {' -> '.join(path)}")
            else:
                self.detail_var.set(step.get("message", "Search complete"))
            return

        delay = max(100, int(1000 * (2.2 - self.speed_var.get())))
        self.animation_job = self.after(delay, self.animate_step)


if __name__ == "__main__":
    app = SearchVisualizer()
    app.mainloop()
