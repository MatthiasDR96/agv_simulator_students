import math


class Node:
    """
            A class containing the Node representation
    """
    
    def __init__(self):
        self.pos = (0, 0)
        self.neighbors = None
        self.name = " "
        self.g = 0
        self.h = 0
        self.parent = None

    def copy_node(self):
        node = Node()
        node.pos = self.pos
        node.neighbors = self.neighbors
        node.name = self.name
        node.g = 0
        node.h = 0
        node.parent = None
        return node

    def move_cost(self, node):
        cost = math.sqrt(math.pow(self.pos[0] - node.pos[0], 2) + math.pow(self.pos[1] - node.pos[1], 2))
        return cost

    def to_string(self):
        return str(str(self.pos[0]) + "," + str(self.pos[1]))

    def is_equal(self, node):
        if self.name == node.name:
            return True
        else:
            return False
