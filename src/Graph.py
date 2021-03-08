import math

import matplotlib.pyplot as plt

class Graph:

    def __init__(self):
        self.__nodes = dict()
        self.__edges = dict()

    @property
    def nodes(self):
        return self.__nodes

    @property
    def edges(self):
        return self.__edges

    def add_node(self, node_location, node_name):
        if node_name not in self.__nodes.keys():
            node = Node(node_location, node_name)
            self.__nodes[node_name] = node

    def add_edge(self, start_node, end_node, length):
        edge = Edge(start_node, end_node, length)
        start_node.add_edge(edge)
        start_node.add_neighbor(end_node.name)
        self.__edges[start_node.name, end_node.name] = edge

    def create_nodes(self, node_locations, node_names):
        for i in range(len(node_locations)):
            self.add_node(node_locations[i], node_names[i])

    def create_edges(self, node_names, node_neighbors):
        for i in range(len(node_names)):
            for j in range(len(node_neighbors[i])):
                start_node = self.__nodes[node_names[i]]
                end_node = self.__nodes[node_neighbors[i][j]]
                length = self.__euclidean_distance(start_node.pos, end_node.pos)
                self.add_edge(start_node, end_node, length)

    def reset_edge_pheromone(self, level=0.01):
        for edge in self.__edges.values():
            edge.pheromone = level

    def evaporate(self):
        for node in self.__nodes.values():
            node.segment.evaporate()
        for edge in self.__edges.values():
            for segment in edge.segments:
                segment.evaporate()

    def remove_reservations(self, agv_id):
        for node in self.__nodes.values():
            node.environmental_agent.remove_reservations(agv_id)

    def print_nodes(self):
        print("\nGraph nodes:")
        for node in self.__nodes.values():
            print("\t", node)
            print("\t\tNeighboring edges: " + str([str(edge) for edge in node.edges]))

    def print_edges(self):
        print("\nGraph edges:")
        for edge in self.__edges.values():
            print("\t", edge)

    def plot(self, ax=None, nodes=(), paths=()):
        if ax is None:
            fig, ax = plt.subplots(1, 1)
        ax.set_title('Layout')
        ax.set_xlabel('x-coordinate (m)')
        ax.set_ylabel('y-coordinate (m)')

        for node in self.__nodes.values():
            ax.plot(node.pos[0], node.pos[1], 'b.', ms=6)
            ax.text(node.pos[0] + 1.5, node.pos[1] + 1.5, node.name)
        for edge in self.__edges.values():
            arrow_x = (edge.start_node.pos[0] + edge.end_node.pos[0]) / 2
            arrow_y = (edge.start_node.pos[1] + edge.end_node.pos[1]) / 2
            delta_x = (edge.end_node.pos[0] - edge.start_node.pos[0]) * 0.01
            delta_y = (edge.end_node.pos[1] - edge.start_node.pos[1]) * 0.01
            ax.plot([edge.start_node.pos[0], edge.end_node.pos[0]], [edge.start_node.pos[1], edge.end_node.pos[1]],
                    'b-', lw=0.5)
            ax.arrow(arrow_x, arrow_y, delta_x, delta_y, length_includes_head=True, head_width=2, head_length=2)

        # To annotate nodes or paths
        for node in nodes:
            node_name = node[0]
            marker = node[1]
            marker_size = node[2]
            ax.plot(self.nodes[node_name].pos[0], self.nodes[node_name].pos[1], marker, ms=marker_size)
        for path in paths:
            nodes_names = path[0]
            marker = path[1]
            marker_size = path[2]
            for i in range(len(nodes_names) - 1):
                ax.plot(self.nodes[nodes_names[i]].pos[0], self.nodes[nodes_names[i]].pos[1], marker, ms=marker_size)
                ax.plot([self.nodes[nodes_names[i]].pos[0], self.nodes[nodes_names[i + 1]].pos[0]],
                        [self.nodes[nodes_names[i]].pos[1], self.nodes[nodes_names[i + 1]].pos[1]], marker,
                        lw=marker_size)
            ax.plot(self.nodes[nodes_names[-1]].pos[0], self.nodes[nodes_names[-1]].pos[1], marker, ms=marker_size)

    @staticmethod
    def __euclidean_distance(a, b):
        return round(math.sqrt(pow(a[1] - b[1], 2) + pow(a[0] - b[0], 2)), 4)

    def plot_schedules(self):

        # Count not empty node schedules
        count_n = 0
        for node in self.nodes.values():
            if not len(node.environmental_agent.reservations) == 0:
                count_n += 1

        if not count_n == 0:
            # Plot schedules
            fig, axes = plt.subplots(nrows=round(math.sqrt(count_n)) + 1,
                                     ncols=round(count_n / round(math.sqrt(count_n))),
                                     gridspec_kw={'hspace': 1.5, 'wspace': 0.5})
            axes = axes.ravel()
            i = 0
            for node in self.nodes.values():
                if not len(node.environmental_agent.reservations) == 0:
                    node.environmental_agent.plot(axes[i])
                    i += 1
        plt.show()


class Node:

    def __init__(self, node_location, node_name):
        self.pos = node_location
        self.name = node_name
        self.edges = []
        self.neighbors = []

        # For astar
        self.g = 0
        self.h = 0
        self.parent = None

    def __str__(self):
        return self.name + ': ' + str(self.pos)

    def __repr__(self):
        return self.name + ': ' + str(self.pos)

    def __hash__(self):
        return hash(str(self))

    def __eq__(self, other):
        return self.name == other.name and self.pos == other.pos

    def copy_node(self):
        node = Node(self.pos, self.name)
        node.neighbors = self.neighbors
        node.edges = self.edges
        node.g = 0
        node.h = 0
        node.parent = None
        return node

    def add_edge(self, edge):
        self.edges.append(edge)

    def add_neighbor(self, node):
        self.neighbors.append(node)

    def move_cost(self, node):
        cost = math.sqrt(math.pow(self.pos[0] - node.pos[0], 2) + math.pow(self.pos[1] - node.pos[1], 2))
        return cost

    def to_string(self):
        return str(str(self.pos[0]) + "," + str(self.pos[1]))


class Edge:

    def __init__(self, start_node, end_node, length, pheromone=0.1):
        self.start_node = start_node
        self.end_node = end_node
        self.length = length
        self.pheromone = pheromone

    def __str__(self):
        return str(self.start_node) + " => " + str(self.end_node)

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        return False
