import math
import numpy as np
from random import randint
import matplotlib.pyplot as plt


class FleetManager:

    def __init__(self, env, global_task_list, global_robot_list, local_task_lists, astar):

        # Attributes
        self.env = env
        self.global_task_list = global_task_list
        self.global_robot_list = global_robot_list
        self.local_task_lists = local_task_lists
        self.update_interval = 1
        self.tasks_not_executing = 0
        self.monitor_tasks_not_executing = []
        self.astar = astar

        # Processes
        self.main = self.env.process(self.main())
        self.monitor_tasks_not_executing_update = self.env.process(self.monitor_tasks_not_executing_update())

        # Initialize
        print("Fleet manager:    Started")

    def main(self):

        print("\n")
        while True:

            # Timeout
            yield self.env.timeout(self.update_interval)

            # Define current status
            robots, tasks, distance_matrix = self.define_current_status()
            #print('FleetManager:     Task list at ' + str(self.env.now) + ': ' + str(
                #[task.to_string() for task in tasks]))
            #print('FleetManager:     Robot list at ' + str(self.env.now) + ': ' + str(
                #[robot.to_string() for robot in robots]))

            # If there are robots and tasks
            if not len(tasks) == 0 and not len(robots) == 0:

                # Optimization
                fitness, solution = optimization(len(robots), len(tasks), distance_matrix)

                # Clear previous assignment
                for task_list in self.local_task_lists:
                    length = len(task_list.items)
                    for i in range(length):
                        task_list.get()

                # Send tasks to AGVs
                for i in range(len(robots)):
                    for j in range(len(tasks)):
                        if solution[i][j] == 1:
                            self.local_task_lists[i].put(tasks[j])
                            #print("FleetManager:     Assigned task " + str(tasks[j].order_number) +" to AGV " + str(robots[i].ID) + " at " + str(self.env.now))

            # Monitor tasks not executing
            self.tasks_not_executing = np.sum([len(self.local_task_lists[i].items) for i in range(len(robots))])

    def define_current_status(self):

        # Define current status
        robots = np.copy(self.global_robot_list.items)
        robots = sorted(robots, key=lambda robot: robot.ID)
        tasks = np.copy(self.global_task_list.items)
        number_robots = len(robots)
        number_tasks = len(tasks)
        distance_matrix = np.zeros((number_robots, number_tasks))
        for i in range(number_robots):
            for j in range(number_tasks):
                distance_ra = calculate_euclidean_distance(robots[i].position, tasks[j].pos_A)
                distance_ab = calculate_euclidean_distance(tasks[j].pos_A, tasks[j].pos_B)
                distance_matrix[i, j] += distance_ra + distance_ab
        return robots, tasks, distance_matrix

    def monitor_tasks_not_executing_update(self):

        while True:
            yield self.env.timeout(1)
            self.monitor_tasks_not_executing.append(self.tasks_not_executing)
            monitor_tasks_not_executing_y = self.monitor_tasks_not_executing
            monitor_tasks_not_executing_x = np.arange(len(monitor_tasks_not_executing_y))
            plt.figure(1)
            plt.subplot(1,4,4)
            plt.subplots_adjust(wspace=1.5)
            plt.title("Tasks not executing")
            plt.ylabel("Amount of tasks not executing")
            plt.xlabel("Simulation time (s)")
            plt.plot(monitor_tasks_not_executing_x, monitor_tasks_not_executing_y)
            plt.draw()
            plt.pause(0.0001)


def calculate_euclidean_distance(a, b):
    return math.sqrt(math.pow((b[0] - a[0]), 2) + math.pow((b[1] - a[1]), 2))


def optimization(number_robots, number_tasks, distance_matrix):

    # Optimization
    iterations = 100000
    best_fitness = 1000
    best_solution = []
    for i in range(iterations):

        # Make solution
        x = np.zeros((number_robots, number_tasks))
        for j in range(number_tasks):
            k = randint(0, number_robots - 1)
            x[k][j] = 1

        # Calculate fitness
        fitness = np.sum(np.multiply(x, distance_matrix))

        # Update
        if fitness < best_fitness:
            best_fitness = fitness
            best_solution = x

    return best_fitness, best_solution
