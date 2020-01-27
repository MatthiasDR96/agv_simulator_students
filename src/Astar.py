import math


class Astar:
    """
        A class containing the Astar path planner
    """
    
    def __init__(self, graph):
        self.graph = graph

    def find_shortest_path(self, start_pos, end_pos):
    
        # Get nodes from locations
        start_node = None
        end_node = None
        for node in self.graph:
            if node.pos[0] == int(start_pos[0]) and node.pos[1] == int(start_pos[1]):
                start_node = node
            if node.pos[0] == end_pos[0] and node.pos[1] == end_pos[1]:
                end_node = node
        if not start_node:
            start_node = self.search_closest_node(start_pos)
        if not end_node:
            end_node = self.search_closest_node(end_pos)
    
        # Compute path
        path = self.astar_search(start_node, end_node)
    
        # Calculate total distance
        distance = 0
        for i in range(len(path) - 1):
            distance += calculate_euclidean_distance(path[i].pos, path[i + 1].pos)
    
        return distance, path

    def astar_search(self, start_node, end_node):
        openset = set()
        closedset = set()
        current = start_node
        openset.add(current)
        while openset:
            current = min(openset, key=lambda o: o.g + o.h)
            if (current.pos[0], current.pos[1]) == (end_node.pos[0], end_node.pos[1]):
                path = []
                while current.parent:
                    path.append(current)
                    current = current.parent
                path.append(current)
                return path[::-1]
            openset.remove(current)
            closedset.add(current)
            for neighbor in current.neighbors:
                if neighbor in closedset:
                    continue
                if neighbor in openset:
                    new_g = current.g + current.move_cost(neighbor)
                    if neighbor.g > new_g:
                        neighbor.g = new_g
                        neighbor.parent = current
                else:
                    neighbor.g = current.g + current.move_cost(neighbor)
                    neighbor.h = heuristic(neighbor, end_node)
                    neighbor.parent = current
                    openset.add(neighbor)
        return None

    def search_closest_node(self, position):
        closest_node = None
        min_distance = 1000
        for node in self.graph:
            distance = calculate_euclidean_distance(position, node.pos)
            if distance < min_distance:
                min_distance = distance
                closest_node = node
        return closest_node


def heuristic(node_a, node_b):
    return math.sqrt(math.pow(node_a.pos[0] - node_b.pos[0], 2) + math.pow(node_a.pos[1] - node_b.pos[1], 2))


def calculate_euclidean_distance(a, b):
    return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))
