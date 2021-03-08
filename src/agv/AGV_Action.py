from src.utils.utils import *


class Action:

    def __init__(self, agv):
        self.agv = agv

    def move_to_node(self, node):
        # Interpolate path
        iterations = 10
        node_position = self.agv.kb['graph'].nodes[node].pos
        distance = calculate_euclidean_distance(self.agv.robot_location, node_position)
        delta_s = distance / iterations
        travel_time = delta_s / self.agv.robot_speed
        alpha = math.atan2((node_position[1] - self.agv.robot_location[1]),
                           (node_position[0] - self.agv.robot_location[0]))
        delta_x = round(delta_s * math.cos(alpha), 2)
        delta_y = round(delta_s * math.sin(alpha), 2)

        # Move between nodes
        for i in range(iterations):
            new_x = round(self.agv.robot_location[0] + delta_x, 2)
            new_y = round(self.agv.robot_location[1] + delta_y, 2)
            yield self.agv.env.process(self.move(new_x, new_y, travel_time))

        # To be sure the agv is exact on the node
        self.agv.robot_location = (node_position[0], node_position[1])
        self.agv.path = self.agv.path[1:]
        self.agv.slots = self.agv.slots[1:]
        self.agv.update_global_robot_list()

    def move(self, x, y, travel_time):
        # Compute heading direction
        self.agv.heading_direction = math.atan2((y - self.agv.robot_location[1]), (x - self.agv.robot_location[0]))
        self.agv.update_global_robot_list()

        # Move
        yield self.agv.env.timeout(travel_time)
        self.agv.robot_location = (x, y)
        self.agv.robot_node = min(self.agv.kb['graph'].nodes.values(),
                                  key=lambda node: calculate_euclidean_distance(node.pos, (x, y))).name
        self.agv.battery_status = round(
            (self.agv.battery_status - self.agv.resource_management.resource_consumption(travel_time)), 2)
        self.agv.travelled_time += float(travel_time)
        self.agv.update_global_robot_list()

    def pick(self):
        yield self.agv.env.timeout(self.agv.task_execution_time)

    def place(self):
        yield self.agv.env.timeout(self.agv.task_execution_time)
