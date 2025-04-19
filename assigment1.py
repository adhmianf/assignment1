import math, heapq, time

cities = {
    "A": (0, 0),
    "B": (2, 1),
    "C": (4, 2),
    "D": (5, 5),
    "E": (1, 4)
}

roads = {
    "A": ["B", "E"],
    "B": ["A", "C"],
    "C": ["B", "D"],
    "D": ["C"],
    "E": ["A", "D"]
}

def euclidean(city1, city2):
    x1, y1 = cities[city1]
    x2, y2 = cities[city2]
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def a_star(start, goal):
    frontier = [(euclidean(start, goal), 0, start, [start])]
    visited = set()
    node_count = 0

    while frontier:
        f, cost, current, path = heapq.heappop(frontier)
        node_count += 1

        if current == goal:
            return path, node_count

        if current in visited:
            continue
        visited.add(current)

        for neighbor in roads[current]:
            if neighbor not in visited:
                g = cost + euclidean(current, neighbor)
                h = euclidean(neighbor, goal)
                heapq.heappush(frontier, (g + h, g, neighbor, path + [neighbor]))

    return None, node_count

def gbfs(start, goal):
    frontier = [(euclidean(start, goal), start, [start])]
    visited = set()
    node_count = 0

    while frontier:
        h, current, path = heapq.heappop(frontier)
        node_count += 1

        if current == goal:
            return path, node_count

        if current in visited:
            continue
        visited.add(current)

        for neighbor in roads[current]:
            if neighbor not in visited:
                heapq.heappush(frontier, (euclidean(neighbor, goal), neighbor, path + [neighbor]))

    return None, node_count

start_city = "A"
goal_city = "D"

# A*
start_time = time.time()
path_astar, nodes_astar = a_star(start_city, goal_city)
time_astar = (time.time() - start_time) * 1000

# GBFS
start_time = time.time()
path_gbfs, nodes_gbfs = gbfs(start_city, goal_city)
time_gbfs = (time.time() - start_time) * 1000

# Output
print("=== A* Search ===")
print("Path:", " -> ".join(path_astar))
print("Visited nodes:", nodes_astar)
print(f"Time: {time_astar:.3f} ms")

print("\n=== GBFS ===")
print("Path:", " -> ".join(path_gbfs))
print("Visited nodes:", nodes_gbfs)
print(f"Time: {time_gbfs:.3f} ms")

print("\n=== Comparison Summary ===")
print(f"A* {'lebih cepat' if time_astar < time_gbfs else 'lebih lambat'} dan {'lebih efisien' if nodes_astar < nodes_gbfs else 'kurang efisien'} dibandingkan GBFS.")
