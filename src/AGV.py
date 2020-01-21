import math

import Comm as comm
from Astar import Astar
from Robot import Robot


class AGV:
    """
        A class containing the intelligence of the AGV agent
    """
    
    def __init__(self, env, agv_params, kb, agv_fm_comm):

        # Attributes
        self.env = env

        # Communication attributes
        self.kb = kb
        self.agv_fm_comm = agv_fm_comm

        # AGV attributes
        self.ID = agv_params['ID']  # Each AGV has an unique ID number
        self.robot_speed = agv_params['robot_speed']  # Constant robot speed
        self.task_execution_time = agv_params['task_execution_time']
        self.robot_location = agv_params['start_location']  # Current AGV location
        self.battery_threshold = agv_params['battery_threshold']  # Battery threshold when the AGV needs to charge
        self.max_charging_time = agv_params['max_charging_time']  # One hour to charge fully

        # Layout attributes
        self.charging_stations = kb['charge_locations']
        self.astar = Astar(self.kb['graph'])  # Astar shortest path planner

        # Updated attributes
        self.heading_direction = 0  # Direction towards which the AGV is driving
        self.path = []  # Current path to execute

        # Monitoring attributes
        self.status = 'IDLE'  # Current AGV status
        self.battery_status = 100  # Current battery status of the AGV

        # Processes
        self.main = self.env.process(self.main())
        self.status_manager = self.env.process(self.status_manager())

        # Initialize
        print("AGV " + str(self.ID) + ":            Started")
        robot = Robot(self.ID, self.robot_location)
        comm.sql_write(self.kb['global_robot_list'], robot)

    def main(self):

        while True:
    
            # Wait for an assigned task
            print("AGV " + str(self.ID) + ":            Waiting for tasks...")
            task = yield self.agv_fm_comm.get()
    
            # Execute task
            print("AGV " + str(self.ID) + ":            Start executing task " + str(task.to_string()) + " at " +
                  str(self.env.now))
            self.status = 'BUSY'
            self.update_robot_status()
    
            # Remove task from global task list and add to executing list
            comm.sql_remove(self.kb['global_task_list'], task.order_number)
            task.robot = self.ID
            comm.sql_write(self.kb['tasks_executing'], task)
    
            # Go to task A
            yield self.env.process(self.execute_path(task.pos_A))
    
            # Perform task A
            yield self.env.timeout(self.task_execution_time)
            task.picked = True
            print("AGV " + str(self.ID) + ":            Picked item of task " + str(task.order_number) + " at " +
                  str(self.env.now))
    
            # Go to task B
            yield self.env.process(self.execute_path(task.pos_B))
    
            # Perform task B
            yield self.env.timeout(self.task_execution_time)
            print("AGV " + str(self.ID) + ":            Dropped item of task " + str(task.order_number) + " at " +
                  str(self.env.now))
    
            # Task executed
            comm.sql_remove(self.kb['tasks_executing'], task.order_number)
    
            # If battery threshold exceeded, go to closest charging station and charge fully
            if self.status == 'EMPTY':
                yield self.env.process(self.charge())

            # Set status to IDLE when task is done or when done charging
            self.status = 'IDLE'
            self.update_robot_status()

    def charge(self):
    
        # Search for closest charging station
        self.status = 'CHARGING'
        self.update_robot_status()
        print("AGV " + str(self.ID) + ":            Goes to closest charging station at " + str(self.env.now))
        closest_charging_station = self.search_closest_charging_station()
    
        # Go to the charging station
        yield self.env.process(self.execute_path(closest_charging_station))
    
        # Compute amount to charge
        tmp_scale = 0.01
        amount_to_charge = 100 - self.battery_status
        charge_time = ((self.max_charging_time * amount_to_charge) / 100) * tmp_scale
        print("AGV " + str(self.ID) + ":            Is charging for " + str(charge_time) + " seconds at " + str(
            self.env.now))
    
        # Charge
        yield self.env.timeout(charge_time)
        self.battery_status = 100
        self.status = 'IDLE'
        self.update_robot_status()

    def execute_path(self, goal_position):
    
        # Compute astar path
        start_position = (int(self.robot_location[0]), int(self.robot_location[1]))
        distance, path = self.astar.find_shortest_path(start_position, goal_position)
        self.path = path[1:]
        self.update_robot_status()
        # print("Path to follow: " + str([node.pos for node in self.path]))

        # Move from node to node
        while len(self.path) > 0:
            node = self.path[0]
            yield self.env.process(self.move_to_node(node))
    
    def move_to_node(self, node):
        
        # Interpolate path
        iterations = 5
        distance = calculate_euclidean_distance(self.robot_location, node.pos)
        delta_s = distance / iterations
        travel_time = delta_s / self.robot_speed
        alpha = math.atan2((node.pos[1] - self.robot_location[1]), (node.pos[0] - self.robot_location[0]))
        delta_x = round(delta_s * math.cos(alpha), 2)
        delta_y = round(delta_s * math.sin(alpha), 2)
        
        # Move between nodes
        for i in range(iterations):
            new_x = round(self.robot_location[0] + delta_x, 2)
            new_y = round(self.robot_location[1] + delta_y, 2)
            yield self.env.process(self.move(new_x, new_y, travel_time))
        
        # To be sure the AGV is exact on the node
        self.robot_location = (node.pos[0], node.pos[1])
        self.path = self.path[1:]
        self.update_robot_status()

    def move(self, x, y, travel_time):
    
        # Compute heading direction
        self.heading_direction = math.atan2((y - self.robot_location[1]), (x - self.robot_location[0]))
        self.update_robot_status()
    
        # Move
        yield self.env.timeout(travel_time)
        self.battery_status = round((self.battery_status - compute_charge_loss(travel_time)), 2)
        self.robot_location = (x, y)
        self.update_robot_status()

    def update_robot_status(self):
        self.kb['global_robot_list'].get(lambda robot_: robot_.ID == self.ID)  # Should be with comm.sql_remove
        robot = Robot(self.ID, self.robot_location, self.heading_direction, self.path, self.status, self.battery_status)
        comm.sql_write(self.kb['global_robot_list'], robot)

    def status_manager(self):
        while True:
            yield self.env.timeout(0.01)  # Sample time
            if not self.status == 'CHARGING':
                if self.battery_status < self.battery_threshold:
                    self.status = 'EMPTY'
                    self.update_robot_status()

    def search_closest_charging_station(self):
        closest_point = None
        min_distance = 1000
        for position in self.charging_stations:
            distance, _ = self.astar.find_shortest_path(position, self.robot_location)
            if distance < min_distance:
                min_distance = distance
                closest_point = position
        return closest_point

    def compute_travel_time(self, pos_a, pos_b):
        distance = calculate_euclidean_distance(pos_a, pos_b)
        total_travel_time = distance / self.robot_speed
        return total_travel_time

    def terminate_task(self, order_number):
        comm.sql_remove(self.kb['global_task_list'], order_number)


def calculate_euclidean_distance(a, b):
    return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))


def compute_charge_loss(travel_time):
    tmp_scale = 20
    charge_loss = tmp_scale * (100.0 / 3600.0) * travel_time  # It takes 1 hour to be empty when driving constantly
    return charge_loss


def almost_equal(a, b):
    epsilon = 0.5
    if abs(a - b) < epsilon:
        return True
    else:
        return False
