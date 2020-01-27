import math


class Action:
    
    def __init__(self, agv):
        self.agv = agv
    
    def move_to_node(self, node):
        # Interpolate path
        iterations = 5
        distance = calculate_euclidean_distance(self.agv.robot_location, node.pos)
        delta_s = distance / iterations
        travel_time = delta_s / self.agv.robot_speed
        alpha = math.atan2((node.pos[1] - self.agv.robot_location[1]), (node.pos[0] - self.agv.robot_location[0]))
        delta_x = round(delta_s * math.cos(alpha), 2)
        delta_y = round(delta_s * math.sin(alpha), 2)
        
        # Move between nodes
        for i in range(iterations):
            new_x = round(self.agv.robot_location[0] + delta_x, 2)
            new_y = round(self.agv.robot_location[1] + delta_y, 2)
            yield self.agv.env.process(self.move(new_x, new_y, travel_time))
        
        # To be sure the AGV is exact on the node
        self.agv.robot_location = (node.pos[0], node.pos[1])
        self.agv.path = self.agv.path[1:]
    
    def move(self, x, y, travel_time):
        # Compute heading direction
        self.agv.heading_direction = math.atan2((y - self.agv.robot_location[1]), (x - self.agv.robot_location[0]))
        
        # Move
        yield self.agv.env.timeout(travel_time)
        self.agv.battery_status = round((self.agv.battery_status - compute_charge_loss(travel_time)), 2)
        self.agv.robot_location = (x, y)
    
    def compute_travel_time(self, pos_a, pos_b):
        distance = calculate_euclidean_distance(pos_a, pos_b)
        total_travel_time = distance / self.agv.robot_speed
        return total_travel_time
    
    def pick(self):
        yield self.agv.env.timeout(self.agv.task_execution_time)
    
    def place(self):
        yield self.agv.env.timeout(self.agv.task_execution_time)


def calculate_euclidean_distance(a, b):
    return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))


def compute_charge_loss(travel_time):
    tmp_scale = 20
    charge_loss = tmp_scale * (100.0 / 3600.0) * travel_time  # It takes 1 hour to be empty when driving constantly
    return charge_loss
