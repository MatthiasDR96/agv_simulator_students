import math

import numpy as np
from Robot import Robot


class AGV:

    def __init__(self, env, id_number, robot_speed, global_task_list, global_robot_list, start_location,
                 local_task_list, astar, task_execution_time, charging_stations, tasks_executing):

        # Attributes
        self.env = env
        self.ID = id_number  # Each AGV has an unique ID number
        self.robot_speed = robot_speed  # Constant robot speed
        self.global_task_list = global_task_list  # AGV needs to remove the task from this list when executing
        self.local_task_list = local_task_list  # Local task list of the AGV, gets refreshed by the fleet manager
        self.global_robot_list = global_robot_list  # AGV updates its internal parameters to this list
        self.robot_location = start_location  # Current AGV location
        self.astar = astar  # Astar shortest path planner
        self.battery_threshold = 20  # Percentage of battery status when the AGV needs to charge
        self.max_charging_time = 3600  # One hour to charge fully
        self.heading_direction = 0  # Direction towards which the AGV is driving
        self.path = []  # Current path to execute
        self.task_execution_time = task_execution_time
        self.charging_stations = charging_stations
        self.tasks_executing = tasks_executing

        # Monitoring attributes
        self.status = 'IDLE'  # Current AGV status
        self.battery_status = 100  # Current battery status of the AGV

        # Processes
        self.main = self.env.process(self.main())
        self.status_manager = self.env.process(self.status_manager())

        # Initialize
        print("AGV " + str(self.ID) + ":            Started")
        robot = Robot(self.ID, self.robot_location)
        self.global_robot_list.put(robot)

    def main(self):

        while True:

            # Dummy Timeout
            yield self.env.timeout(0.01)

            # Sort the tasks on shortest distance to robot location
            tasks = np.copy(self.local_task_list.items)
            tasks = sorted(tasks, key=lambda task: self.get_first_task(task))

            # Pick fist task
            for item in tasks:

                # Execute task
                print("AGV " + str(self.ID) + ":            Start executing task " + str(item.to_string()) + " at " +
                      str(self.env.now))
                self.status = 'BUSY'
                self.update_robot_status()

                # Remove task from local and global task list and add to executing list
                self.local_task_list.get(lambda task: task.order_number == item.order_number)
                self.global_task_list.get(lambda task: task.order_number == item.order_number)
                item.robot = self.ID
                self.tasks_executing.put(item)

                # Go to task A
                yield self.env.process(self.execute_path(item.pos_A))

                # Perform task A
                yield self.env.timeout(self.task_execution_time)
                item.picked = True
                print("AGV " + str(self.ID) + ":            Picked item of task " + str(item.order_number) + " at " +
                      str(self.env.now))

                # Go to task B
                yield self.env.process(self.execute_path(item.pos_B))

                # Perform task B
                yield self.env.timeout(self.task_execution_time)
                print("AGV " + str(self.ID) + ":            Dropped item of task " + str(item.order_number) + " at " +
                      str(self.env.now))

                # Task executed
                self.tasks_executing.get(lambda task: task.order_number == item.order_number)

                # If battery threshold exceeded, finish task and charge
                if self.status == 'EMPTY':
                    break
                else:
                    self.status = 'IDLE'
                    self.update_robot_status()

            if self.status == 'EMPTY':
                # Go to closest charging station and charge fully
                yield self.env.process(self.charge())

    def charge(self):

        # Search for closest charging station and go to that station
        self.status = 'CHARGING'
        self.update_robot_status()
        print("AGV " + str(self.ID) + ":            Goes to closest charging station at " + str(self.env.now))
        closest_charging_station = self.search_closest_charging_station()

        yield self.env.process(self.execute_path(closest_charging_station))

        # Compute amount to charge and charge
        tmp_scale = 0.01
        amount_to_charge = 100 - self.battery_status
        charge_time = ((self.max_charging_time * amount_to_charge) / 100) * tmp_scale
        print("AGV " + str(self.ID) + ":            Is charging for " + str(charge_time) + " seconds at " + str(
            self.env.now))
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
        self.heading_direction = math.atan2((y - self.robot_location[1]), (x - self.robot_location[0]))
        self.update_robot_status()
        yield self.env.timeout(travel_time)
        self.battery_status = round((self.battery_status - compute_charge_loss(travel_time)), 2)
        self.robot_location = (x, y)
        self.update_robot_status()

    def update_robot_status(self):
        self.global_robot_list.get(lambda robot_: robot_.ID == self.ID)
        robot = Robot(self.ID, self.robot_location, self.heading_direction, self.path, self.status, self.battery_status)
        self.global_robot_list.put(robot)

    def status_manager(self):
        while True:
            yield self.env.timeout(0.01)
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
        self.global_task_list.get(lambda task: task.order_number == order_number)
        # print("AGV " + str(self.ID) + ":            terminated task with order number: " + str(order_number))

    def get_first_task(self, task):
        distance, _ = self.astar.find_shortest_path(task.pos_A, self.robot_location)
        priority = task.priority
        result = distance * priority
        return result


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
