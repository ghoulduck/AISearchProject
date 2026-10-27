from collections import deque
import heapq
import tkinter as tk
from tkinter import ttk, messagebox

# Original project logic is preserved below.
# These are the same graph-search ideas the project was built around,
# while adding a windowed visual interface and A* support.
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
    x1, y1 = NODE_POSITIONS[node]
    x2, y2 = NODE_POSITIONS[goal]
    return abs(x1 - x2) + abs(y1 - y2)


# Original BFS logic retained and kept intact.
def bfs(start, goal, mygraph):
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


# Original DFS logic retained and kept intact.
def dfs(node, goal, mygraph, visited=None, path=None):
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


# A* pathfinding added without removing BFS/DFS.
def astar(start, goal, graph):
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


# Step-by-step visualizers for the live window.
def bfs_steps(start, goal, graph):
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


class SearchVisualizer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AI Search Project")
        self.geometry("1000x720")
        self.configure(bg="#f2f4f8")

        self.current_steps = []
        self.current_index = 0
        self.animation_job = None
        self.current_mode = "BFS"

        self.start_var = tk.StringVar(value="A")
        self.goal_var = tk.StringVar(value="G")
        self.status_var = tk.StringVar(value="Ready")
        self.detail_var = tk.StringVar(value="Select nodes and run BFS, DFS, or A*.")

        top = ttk.Frame(self, padding=12)
        top.pack(fill="x")

        ttk.Label(top, text="Start Node:", font=("Segoe UI", 10, "bold")).grid(row=0, column=0, padx=(0, 8), sticky="w")
        ttk.Combobox(top, textvariable=self.start_var, values=sorted(mygraph.keys()), state="readonly", width=8).grid(row=0, column=1, padx=(0, 16))

        ttk.Label(top, text="Goal Node:", font=("Segoe UI", 10, "bold")).grid(row=0, column=2, padx=(0, 8), sticky="w")
        ttk.Combobox(top, textvariable=self.goal_var, values=sorted(mygraph.keys()), state="readonly", width=8).grid(row=0, column=3, padx=(0, 16))

        ttk.Button(top, text="Run BFS", command=lambda: self.run_search("BFS")).grid(row=0, column=4, padx=(0, 8))
        ttk.Button(top, text="Run DFS", command=lambda: self.run_search("DFS")).grid(row=0, column=5, padx=(0, 8))
        ttk.Button(top, text="Run A*", command=lambda: self.run_search("A* ")).grid(row=0, column=6, padx=(0, 8))
        ttk.Button(top, text="Reset", command=self.reset_view).grid(row=0, column=7)

        info = ttk.Frame(self, padding=(12, 0, 12, 12))
        info.pack(fill="x")
        ttk.Label(info, textvariable=self.status_var, font=("Segoe UI", 11, "bold"), foreground="#1a73e8").pack(anchor="w")
        ttk.Label(info, textvariable=self.detail_var, wraplength=950, justify="left").pack(anchor="w", pady=(4, 0))

        self.canvas = tk.Canvas(self, width=900, height=520, bg="#ffffff", highlightthickness=1, highlightbackground="#d0d7de")
        self.canvas.pack(fill="both", padx=12, pady=(0, 12), expand=True)

        self.draw_graph()
        self.reset_view()

    def reset_view(self):
        if self.animation_job is not None:
            self.after_cancel(self.animation_job)
            self.animation_job = None

        self.current_steps = []
        self.current_index = 0
        self.status_var.set("Ready")
        self.detail_var.set("Select nodes and run BFS, DFS, or A*.")
        self.draw_graph()

    def draw_graph(self, current=None, visited=None, queue=None, stack=None, path=None):
        self.canvas.delete("all")
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

    def run_search(self, mode):
        if self.animation_job is not None:
            self.after_cancel(self.animation_job)
            self.animation_job = None

        start = self.start_var.get()
        goal = self.goal_var.get()

        if start not in mygraph or goal not in mygraph:
            messagebox.showerror("Invalid selection", "Please pick valid nodes from the graph.")
            return

        self.current_mode = mode.strip()

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
        if not self.current_steps:
            self.status_var.set("No steps to display")
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

        if step.get("type") in {"found", "not_found"}:
            if path:
                self.detail_var.set(f"Path: {' -> '.join(path)}")
            else:
                self.detail_var.set(step.get("message", "Search complete"))
            return

        self.animation_job = self.after(700, self.animate_step)


if __name__ == "__main__":
    app = SearchVisualizer()
    app.mainloop()
