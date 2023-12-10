import osmnx as ox
import folium
import networkx as nx
import math

# Define the A* algorithm
def astar(graph, start, goal, heuristic, weight='length'):
    open_set = []
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    open_set.append((start, f_score[start]))

    while open_set:
        current, _ = min(open_set, key=lambda x: x[1])

        if current == goal:
            path = [current]
            while current != start:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        open_set.remove((current, f_score[current]))

        for neighbor in graph.neighbors(current):
            edge_data = graph.get_edge_data(current, neighbor)
            if edge_data is None:
                continue

            tentative_g_score = g_score[current] + edge_data.get(weight, 1)

            if tentative_g_score < g_score.get(neighbor, float('inf')):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if neighbor not in open_set:
                    open_set.append((neighbor, f_score[neighbor]))

    return None

# Rest of the code remains the same...


# Define the Euclidean distance heuristic
def euclidean_distance(node1, node2):
    x1, y1 = graph.nodes[node1]['x'], graph.nodes[node1]['y']
    x2, y2 = graph.nodes[node2]['x'], graph.nodes[node2]['y']
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Read data from the OSM file
graph = ox.graph_from_xml('D:\map.osm')

# Specific nodes to find the shortest path between
start_node = list(graph.nodes())[0]
end_node = list(graph.nodes())[10]

# Find the shortest path using the custom A* algorithm
shortest_path = astar(graph, start_node, end_node, heuristic=euclidean_distance, weight='length')

# Create a map to visualize the shortest path
my_map = folium.Map(location=(21.015840, 105.847386), zoom_start=15)

# Draw the shortest path on the map
for i in range(len(shortest_path) - 1):
    u, v = shortest_path[i], shortest_path[i + 1]
    x_u, y_u = graph.nodes[u]['x'], graph.nodes[u]['y']
    x_v, y_v = graph.nodes[v]['x'], graph.nodes[v]['y']

    folium.PolyLine(locations=[(y_u, x_u), (y_v, x_v)], color='green').add_to(my_map)

# Save the map to an HTML file
my_map.save('./shortest_path_custom_astar.html')
