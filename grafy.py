import sys
import random
from collections import deque

class Graph:
    def __init__(self, nodes, representation):
        self.nodes = nodes
        self.representation = representation
        self.graph = self._init_graph()

    def _init_graph(self):
        if self.representation == "matrix":
            return [[0] * self.nodes for _ in range(self.nodes)]
        elif self.representation == "list":
            return [[] for _ in range(self.nodes)]
        elif self.representation == "table":
            return []
        else:
            raise ValueError("Unknown representation")

    def add_edge(self, u, v):
        u -= 1
        v -= 1
        if self.representation == "matrix":
            self.graph[u][v] = 1
        elif self.representation == "list":
            self.graph[u].append(v)
        elif self.representation == "table":
            self.graph.append((u, v))

    def print_graph(self):
        if self.representation == "matrix":
            print("   | " + " ".join(str(i+1) for i in range(self.nodes)))
            print("---+" + "-"*(2*self.nodes))
            for i, row in enumerate(self.graph):
                print(f"{i+1:>2} | " + " ".join(str(x) for x in row))
        elif self.representation == "list":
            for i, neighbors in enumerate(self.graph):
                print(f"{i+1}> {' '.join(str(v+1) for v in neighbors)}")
        elif self.representation == "table":
            for u, v in self.graph:
                print(f"{u+1} -> {v+1}")

    def has_edge(self, u, v):
        u -= 1
        v -= 1
        if self.representation == "matrix":
            return self.graph[u][v] == 1
        elif self.representation == "list":
            return v in self.graph[u]
        elif self.representation == "table":
            return (u, v) in self.graph

    def neighbors(self, u):
        if self.representation == "matrix":
            return [v for v in range(self.nodes) if self.graph[u][v] == 1]
        elif self.representation == "list":
            return self.graph[u]
        elif self.representation == "table":
            return [v for (x, v) in self.graph if x == u]

    def bfs(self, start):
        start -= 1
        visited = [False] * self.nodes
        queue = deque([start])
        visited[start] = True
        result = []

        while queue:
            u = queue.popleft()
            result.append(u + 1)
            for v in self.neighbors(u):
                if not visited[v]:
                    visited[v] = True
                    queue.append(v)

        print("inline:", " ".join(map(str, result)))

    def dfs(self, start):
        start -= 1
        visited = [False] * self.nodes
        result = []

        def dfs_visit(u):
            visited[u] = True
            result.append(u + 1)
            for v in self.neighbors(u):
                if not visited[v]:
                    dfs_visit(v)

        dfs_visit(start)
        print("inline:", " ".join(map(str, result)))
    
    def topo_kahn(self):
        in_degree = [0] * self.nodes
        for u in range(self.nodes):
            for v in self.neighbors(u):
                in_degree[v] += 1

        queue = deque([u for u in range(self.nodes) if in_degree[u] == 0])
        result = []

        while queue:
            u = queue.popleft()
            result.append(u + 1)
            for v in self.neighbors(u):
                in_degree[v] -= 1
                if in_degree[v] == 0:
                    queue.append(v)

        if len(result) != self.nodes:
            print("Error: graph has at least one cycle (Kahn).")
        else:
            print("Topological order (Kahn):", " ".join(map(str, result)))

    def topo_tarjan(self):
        temp_mark = [False] * self.nodes
        perm_mark = [False] * self.nodes
        result = []
        cycle_detected = [False]

        def visit(u):
            if perm_mark[u]:
                return
            if temp_mark[u]:
                cycle_detected[0] = True
                return
            temp_mark[u] = True
            for v in self.neighbors(u):
                visit(v)
            temp_mark[u] = False
            perm_mark[u] = True
            result.append(u + 1)

        for u in range(self.nodes):
            if not perm_mark[u]:
                visit(u)

        if cycle_detected[0]:
            print("Error: graph has at least one cycle (Tarjan).")
        else:
            result.reverse()
            print("Topological order (Tarjan):", " ".join(map(str, result)))

def generate_random_graph(graph, saturation_percent):
    """Generuje losowy graf skierowany o zadanej procentowej saturacji (0–100)."""
    if not (0 <= saturation_percent <= 100):
        raise ValueError("Saturacja musi być w zakresie 0–100")

    saturation = saturation_percent / 100.0
    total_possible_edges = graph.nodes * (graph.nodes - 1)
    target_edges = int(total_possible_edges * saturation)

    all_possible_edges = [(u, v) for u in range(graph.nodes) for v in range(graph.nodes) if u != v]
    selected_edges = random.sample(all_possible_edges, min(target_edges, len(all_possible_edges)))

    for u, v in selected_edges:
        graph.add_edge(u + 1, v + 1)

def user_input_edges(graph):
    for i in range(1, graph.nodes + 1):
        line = input(f"{i}> ").strip()
        if line:
            for succ in map(int, line.split()):
                graph.add_edge(i, succ)

def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ["--generate", "--user-provided"]:
        print("Usage: ./program.py --generate | --user-provided")
        return

    rep = input("type> ").strip().lower()
    nodes = int(input("nodes> "))

    graph = Graph(nodes, rep)

    if sys.argv[1] == "--generate":
        try:
            saturation_input = int(input("saturation (0–100)%> "))
            generate_random_graph(graph, saturation_input)
        except ValueError as e:
            print("Invalid input for saturation:", e)
            return
    else:
        user_input_edges(graph)

    print("Type 'Help' for list of commands.")

    while True:
        try:
            action = input("action> ").strip().lower()
            if action == "help":
                print("Help - This message")
                print("Print - Print graph")
                print("Find - Check if edge between two nodes is in the graph")
                print("BFS - Breath-first search - print BFS search")
                print("DFS - Depth-first search - print DFS search")
                print("Topo-kahn - Sorting")
                print("Topo-tarjan - Sorting")
                print("Exit or quit - Leave program")
            elif action == "print":
                graph.print_graph()
            elif action == "find":
                u = int(input("from> "))
                v = int(input("to> "))
                exists = graph.has_edge(u, v)
                if exists:
                    print(f"True: edge ({u},{v}) exists in the Graph!")
                else:
                    print(f"False: edge ({u},{v}) does not exist in the Graph!")
            elif action == "bfs":
                graph.bfs(1)
            elif action == "dfs":
                graph.dfs(1)
            elif action == "topo-kahn":
                graph.topo_kahn()
            elif action == "topo-tarjan":
                graph.topo_tarjan()
            elif action in ("exit", "quit"):
                break
            else:
                print("Unknown action. Try: Print, Find, BFS, DFS, Topo-kahn, Topo-tarjan, Exit")
        except Exception as e:
            print("Error:", e)

if __name__ == "__main__":
    main()
