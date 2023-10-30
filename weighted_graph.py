import heapq

"""
Proposition: Djikstra's Algorithm
Often Used: Networking; to find closest nodes
Further Reference: https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
"""
class WeightedGraph:
    infinity_number = float('inf')
    def __init__(self):
        self.graph = {}  # Initialize an empty graph

    # Method to add an edge to the graph with a specified weight
    def add_edge(self, start, end, weight):
        if start not in self.graph:
            self.graph[start] = []  # Initialize an empty list for the start node if it's not already in the graph

        self.graph[start].append((end, weight))  # Add the end node and weight to the start node's list

        if end not in self.graph:
            self.graph[end] = []  # Initialize an empty list for the end node if it's not already in the graph

        self.graph[end].append((start, weight))  # Since the graph is undirected, add the reverse edge

    # Dijkstra's algorithm to find the shortest path from the start to the end
    def dijkstra_algorithm(self, start, end):
        priority_queue = [(0, start)]  # Initialize a priority queue with the start node and a distance of 0
        shortest_distances = {node: self.infinity_number for node in self.graph}  # Initialize all distances to infinity
        previous_nodes = {}  # Initialize a dictionary to track the previous node in the shortest path
        shortest_distances[start] = 0  # Set the distance of the start node to 0

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)  # Get the node with the shortest distance

            if current_distance > shortest_distances[current_node]:
                continue  # Skip this node if a shorter path has already been found

            for neighbor, weight in self.graph[current_node]:
                distance = current_distance + weight
                if distance < shortest_distances[neighbor]:
                    shortest_distances[neighbor] = distance  # Update the shortest distance
                    previous_nodes[neighbor] = current_node  # Update the previous node
                    heapq.heappush(priority_queue, (distance, neighbor))  # Add to the priority queue

        if end not in previous_nodes:
            return None, self.infinity_number  # If there's no path to the end node, return None and infinity

        path = []
        current = end
        while current:
            path.append(current)
            current = previous_nodes.get(current)  # Reconstruct the path by backtracking

        path.reverse()
        path_cost = shortest_distances[end]

        return path, path_cost

# Create a weighted graph from values provided in the sample map image
graph = WeightedGraph()
graph.add_edge("Kasungu", "Ntchisi", 66)
graph.add_edge("Ntchisi", "Nkhotakota", 66)
graph.add_edge("Ntchisi", "Dowa", 38)
graph.add_edge("Kasungu", "Dowa", 117)
graph.add_edge("Nkhotakota", "Salima", 12)
graph.add_edge("kasungu", "Mchinji", 114)
graph.add_edge("Mchinji", "Lilongwe", 109)
graph.add_edge("Dowa", "Lilongwe", 55)
graph.add_edge("Dowa", "Salima", 67)
graph.add_edge("Salima", "Dedza", 96)
graph.add_edge("Dedza", "Ntcheu", 74)
graph.add_edge("Lilongwe", "Dedza", 92)

# Calculating the shortest path from Kasungu to Ntcheu
start_node = "Kasungu"
end_node = "Ntcheu"
shortest_path, path_cost = graph.dijkstra_algorithm(start_node, end_node)

if shortest_path:
    print(f"Shortest path from {start_node} to {end_node}: {shortest_path}")
    print(f"Total distance of the path: {path_cost}")
else:
    print("No path found between the specified districts.")
