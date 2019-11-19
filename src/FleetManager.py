import math

import numpy as np
from ortools.sat.python import cp_model


class FleetManager:

    def __init__(self, env, global_task_list, global_robot_list, local_task_lists, astar):

        # Attributes
        self.env = env
        self.global_task_list = global_task_list  # Fleet manager uses this list as start to assign tasks
        self.global_robot_list = global_robot_list  # Fleet manager uses this list to obtain AGV information
        self.local_task_lists = local_task_lists  # Fleet manager uses this list to send the assignments to the AGVs
        self.astar = astar  # Astar shortest path finder
        self.fitness_file = open("../logfiles/fitness_values.txt", "w")

        # Process
        self.main = self.env.process(self.main())

        # Initialize
        print("Fleet manager:    Started")

    def main(self):

        print("\n")
        while True:

            # Timeout
            yield self.env.timeout(1)  # Sample time

            # Define current status
            robots, tasks, cost_matrix = self.define_current_status()
            # print('FleetManager:     Task list at ' + str(self.env.now) + ': ' + str(
            # [task.to_string() for task in tasks]))
            # print('FleetManager:     Robot list at ' + str(self.env.now) + ': ' + str(
            # [robot.to_string() for robot in robots]))

            # If there are robots and tasks
            if not len(tasks) == 0 and not len(robots) == 0:

                # Optimization
                fitness, solution = optimization(len(robots), len(tasks), cost_matrix)

                # Log fitness values for each assignment
                self.fitness_file.write(str(self.env.now) + "," + str(fitness) + ",\n")

                # Clear previous assignment
                for task_list in self.local_task_lists:
                    length = len(task_list.items)
                    for i in range(length):
                        task_list.get()

                # Send tasks to AGVs
                for i in range(len(robots)):
                    for j in range(len(tasks)):
                        if solution[i][j] == 1:
                            self.local_task_lists[robots[i].ID - 1].put(tasks[j])
                            # print("FleetManager:     Assigned task " + str(tasks[j].order_number) +" to AGV " +
                            # str(robots[i].ID) + " at " + str(self.env.now))
            else:
                # Log fitness values for no assignment (0.0)
                self.fitness_file.write(str(self.env.now) + "," + str(0.0) + ",\n")

    def define_current_status(self):

        # Copy robot list
        robots = np.copy(self.global_robot_list.items)
        robots = sorted(robots, key=lambda robot_: robot_.ID)

        # Remove robots which are charging or have troubles
        robots_ = []
        for robot in robots:
            if not robot.status == 'EMPTY' and not robot.status == 'CHARGING':
                robots_.append(robot)

        # Copy global task list
        tasks = np.copy(self.global_task_list.items)
        number_robots = len(robots_)
        number_tasks = len(tasks)

        # Compute cost matrix
        cost_matrix = np.zeros((number_robots, number_tasks))
        for i in range(number_robots):
            for j in range(number_tasks):
                distance_ra, _ = self.astar.find_shortest_path(robots_[i].position, tasks[j].pos_A)
                distance_ab, _ = self.astar.find_shortest_path(tasks[j].pos_A, tasks[j].pos_B)
                cost_matrix[i, j] = (distance_ra + distance_ab)
        return robots_, tasks, cost_matrix


def calculate_euclidean_distance(a, b):
    return math.sqrt(math.pow((b[0] - a[0]), 2) + math.pow((b[1] - a[1]), 2))


def optimization(number_robots, number_tasks, distance_matrix):
    # Cost matrix
    cost = distance_matrix.astype(int)

    # Optimization model
    model = cp_model.CpModel()

    # Variable matrix x
    x = []
    for i in range(number_robots):
        t = []
        for j in range(number_tasks):
            t.append(model.NewIntVar(0, 1, "x[%i,%i]" % (i, j)))
        x.append(t)

    # Constraints: Each task is assigned to exact one robot.
    [model.Add(sum(x[i][j] for i in range(number_robots)) == 1) for j in range(number_tasks)]

    # Solve
    model.Minimize(sum([np.dot(x_row, cost_row) for (x_row, cost_row) in zip(x, cost)]))
    solver = cp_model.CpSolver()
    status = solver.Solve(model)

    best_solution = []
    if status == cp_model.OPTIMAL:
        for i in range(number_robots):
            t = []
            for j in range(number_tasks):
                t.append(solver.Value(x[i][j]))
            best_solution.append(t)
    else:
        print("No solution found.")

    return solver.ObjectiveValue(), best_solution
