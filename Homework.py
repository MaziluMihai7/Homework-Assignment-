import heapq
import itertools

class TSP:
    def __init__(self, distances):
        self.distances = distances
        self.num_cities = len(distances)
        self.initial_city = 0  # Starting city

    # Helper function to calculate the total cost of a path
    def path_cost(self, path):
        total_cost = 0
        for i in range(len(path) - 1):
            total_cost = max(total_cost, self.distances[path[i]][path[i+1]])
        total_cost = max(total_cost, self.distances[path[-1]][path[0]])  # Return to initial city
        return total_cost

    # Breadth-First Search
    def bfs(self):
        min_cost = float('inf')
        best_path = None
        queue = [(self.initial_city, [self.initial_city])]
        while queue:
            current_city, path = queue.pop(0)
            if len(path) == self.num_cities:
                cost = self.path_cost(path)
                if cost < min_cost:
                    min_cost = cost
                    best_path = path
            else:
                for city in range(self.num_cities):
                    if city not in path:
                        queue.append((city, path + [city]))
        return best_path

    # Least-Cost (Uniform Cost) Search
    def least_cost_search(self):
        min_cost = float('inf')
        best_path = None
        queue = [(0, [0])]  # (cost, path)
        while queue:
            cost, path = heapq.heappop(queue)
            if len(path) == self.num_cities:
                if cost < min_cost:
                    min_cost = cost
                    best_path = path
            else:
                for city in range(self.num_cities):
                    if city not in path:
                        new_cost = max(cost, self.distances[path[-1]][city])
                        heapq.heappush(queue, (new_cost, path + [city]))
        return best_path

    # Heuristic function for A* Search
    def heuristic(self, path):
        # Heuristic: Minimum distance from the last city to any unvisited city
        unvisited = set(range(self.num_cities)) - set(path)
        if not unvisited:
            return 0
        return min(self.distances[path[-1]][city] for city in unvisited)

    # A* Search
    def a_star_search(self):
        min_cost = float('inf')
        best_path = None
        queue = [(0, [0])]  # (cost + heuristic, path)
        while queue:
            cost, path = heapq.heappop(queue)
            if len(path) == self.num_cities:
                if cost < min_cost:
                    min_cost = cost
                    best_path = path
            else:
                for city in range(self.num_cities):
                    if city not in path:
                        new_cost = max(cost, self.distances[path[-1]][city])
                        heapq.heappush(queue, (new_cost + self.heuristic(path + [city]), path + [city]))
        return best_path

# Example usage:
distances = [
    [0, 10, 15, 20],
    [10, 0, 35, 25],
    [15, 35, 0, 30],
    [20, 25, 30, 0]
]

tsp_solver = TSP(distances)

# Breadth-First Search
print("BFS:", tsp_solver.bfs())

# Least-Cost (Uniform Cost) Search
print("Least-Cost Search:", tsp_solver.least_cost_search())

# A* Search
print("A* Search:", tsp_solver.a_star_search())
