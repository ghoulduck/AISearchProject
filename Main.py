def dfs(node, goal, mygraph, visited, path):
    if node == goal:
        return path
    if node not in visited:
        visited.append(node)
        for inner in mygraph[node]:
            path.append()
            dfs(inner, goal, mygraph, visited, path)
    return None


def bfs(start, goal, mygraph):
    queue = [start]
    visited = []

    while queue:
        print(f"Current Queue: {queue}")
        path = queue.pop(0)
        node = path[-1]

        if node in visited:
            print(f"Node {node} already visited by algorithm")
            continue

        visited.append(node)
        print(f"Node {node} added to visited")

        if node == goal:
            return path

        for child in mygraph[node]:
            if child not in visited:
                queue.append(path + [child])

    return None