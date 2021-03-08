import math
from copy import deepcopy


def find_shortest_path(graph, start_node, end_node):
    """
        Input:
            - Total layout graph
            - Start node name
            - End node name
        Output:
            - Shortest path names
            - Total distance in meters
        Default output:
            - None
            - None
    """

    # Tak deepcopy of graph
    graph = deepcopy(graph)

    # Get nodes from node names
    start_node = graph.nodes[start_node]
    end_node = graph.nodes[end_node]

    # Init
    openset = set()
    closedset = set()
    current = start_node
    openset.add(current)

    # Loop
    while openset:

        # Take node with least cost as next node
        current = min(openset, key=lambda o: o.g + o.h)

        # End criterium
        if current.name == end_node.name:
            path = []
            distance = 0
            while current.parent:
                path.append(current.name)
                distance += graph.edges[current.name, current.parent.name].length
                current = current.parent
            path.append(current.name)
            return path[::-1], distance

        # Move to next node
        openset.remove(current)
        closedset.add(current)

        # Explore neighbors
        for neighbor_name in current.neighbors:
            neighbor = graph.nodes[neighbor_name]
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
    return None, None


def heuristic(node_a, node_b):
    return math.sqrt(math.pow(node_a.pos[0] - node_b.pos[0], 2) + math.pow(node_a.pos[1] - node_b.pos[1], 2))
