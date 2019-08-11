from Robot import Robot
import math
import numpy as np
import matplotlib.pyplot as plt

plt.ion()


class AGV:

    def __init__(self, env, id_number, robot_speed, global_task_list, global_robot_list, start_location,
                 local_task_list, astar):

        # Attributes
        self.env = env
        self.ID = id_number
        self.robot_speed = robot_speed
        self.global_task_list = global_task_list
        self.local_task_list = local_task_list
        self.global_robot_list = global_robot_list
        self.robot_location = start_location
        self.operating_monitor = []
        self.status = 'IDLE'
        self.astar = astar
        self.collision_threshold = 0.5
        self.collision = False
        self.heading_direction = 0
        self.path = []
        self.battery_status = 100

        # Processes
        self.main = self.env.process(self.main())
        self.operating_monitor_update = self.env.process(self.operating_monitor_update())
        self.collision_monitor = self.env.process(self.collision_monitor())

        # Initialize
        print("AGV " + str(self.ID) + ":            Started")
        robot = Robot(self.ID, self.robot_location)
        self.global_robot_list.put(robot)

    def main(self):

        while True:

            # Timeout
            yield self.env.timeout(0.01)

            # Loop over local task list
            for item in self.local_task_list.items:

                # Execute task
                print("AGV " + str(self.ID) + ":            Start executing task " + str(item.order_number) + " at " +
                      str(self.env.now))
                self.status = 'BUSY'

                # Remove task from local and global task list
                self.local_task_list.get(lambda task: task.order_number == item.order_number)
                self.global_task_list.get(lambda task: task.order_number == item.order_number)

                # Go to task A
                yield self.env.process(self.execute_path(item.pos_A))

                # Perform task A
                yield self.env.timeout(2)
                print("AGV " + str(self.ID) + ":            Picked item of task " + str(item.order_number) + " at " +
                      str(self.env.now))

                # Go to task B
                yield self.env.process(self.execute_path(item.pos_B))

                # Perform task B
                yield self.env.timeout(2)
                print("AGV " + str(self.ID) + ":            Dropped item of task " + str(item.order_number) + " at " +
                      str(self.env.now))

                # Task executed
                self.status = 'IDLE'

    def update_robot_status(self):
            self.global_robot_list.get(lambda robot: robot.ID == self.ID)
            robot = Robot(self.ID, self.robot_location, self.heading_direction, self.path, self.status, self.battery_status)
            self.global_robot_list.put(robot)

    def operating_monitor_update(self):
        while True:
            yield self.env.timeout(1)
            plt.figure(1)
            self.operating_monitor.append(self.status == 'BUSY')
            operating_monitor_y = self.operating_monitor
            operating_monitor_x = np.arange(len(operating_monitor_y))
            plt.subplot(1,4,self.ID)
            plt.title("AGV" + str(self.ID))
            plt.xlabel("Simulation time (s)")
            plt.ylabel("IDLE (0), BUSY (1)")
            plt.plot(operating_monitor_x, operating_monitor_y)
            plt.draw()
            plt.pause(0.0001)

    def collision_monitor(self):
        while True:
            yield self.env.timeout(0.01)
            robots = self.global_robot_list.items
            robots = sorted(robots, key=lambda robot: robot.ID)
            distances = [100]
            for robot in robots:
                if not robot.ID == self.ID:
                    # True if there is a robot in front of us
                    same_heading = almost_equal((math.atan2((robot.position[1] - self.robot_location[1]), (robot.position[0] - self.robot_location[0]))), self.heading_direction)
                    if same_heading:
                        distances.append(calculate_euclidean_distance(self.robot_location, robot.position))
            if np.min(distances) < self.collision_threshold:
                self.collision = True
            else:
                self.collision = False

    def execute_path(self, goal_position):
        # Compute astar path
        start_position = (int(self.robot_location[0]), int(self.robot_location[1]))
        distance, path = self.astar.find_shortest_path(start_position, goal_position)
        self.path = path
        self.update_robot_status()
        #print("Path to follow: " + str([node.pos for node in path]))

        # Move from node to node
        for i in range(len(path[1:])):
            node = path[i+1]

            # Compute delta_x and delta_y
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
            self.update_robot_status()

        self.path = []
        self.update_robot_status()

    def move(self, x, y, travel_time):
        self.heading_direction = math.atan2((y - self.robot_location[1]), (x - self.robot_location[0]))
        if not self.collision:
            yield self.env.timeout(travel_time)
            self.robot_location = (x, y)
            self.update_robot_status()
        else:
            print("AGV " + str(self.ID) + " stopped at " + str(self.env.now))
            while self.collision:
                yield self.env.timeout(1)
            print("AGV " + str(self.ID) + " continued again at " + str(self.env.now))

    def compute_travel_time(self, pos_a, pos_b):
        distance = calculate_euclidean_distance(pos_a, pos_b)
        total_travel_time = distance / self.robot_speed
        return total_travel_time

    def terminate_task(self, order_number):
        self.global_task_list.get(lambda task: task.order_number == order_number)
        print("AGV " + str(self.ID) + ":            terminated task with order number: " + str(order_number))


def calculate_euclidean_distance(a, b):
    return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))


def almost_equal(a, b):
    epsilon = 0.5
    if abs(a - b) < epsilon:
        return True
    else:
        return False
