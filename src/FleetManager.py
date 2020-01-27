import math

import numpy as np
from ortools.sat.python import cp_model

from Astar import Astar
from Comm import Comm


class FleetManager:
    """
            A class containing the intelligence of the Fleetmanager agent
    """

    def __init__(self, env, kb, agv_fm_comm, agv_to_fm_comm):
    
        # Simulation environment
        self.env = env
    
        # Communication attributes
        self.kb = kb
        self.agv_fm_comm = agv_fm_comm
        self.agv_to_fm_comm = agv_to_fm_comm
        self.comm = Comm()
        
        # Other attributes
        self.astar = Astar(self.kb['graph'])  # Astar shortest path finder
        self.fitness_file = open("../logfiles/fitness_values.txt", "w")
    
        # Process
        self.main = self.env.process(self.main())
    
        # Initialize
        print("Fleet manager:           Started")

    def main(self):
    
        print("\n")
        while True:
    
            # Timeout
            yield self.env.timeout(1)  # Sample time
    
            # Define current status
            available_robots, tasks_to_execute, cost_matrix = self.define_current_status()
            # print("Fleet manager:    Available robots: " + str(len(available_robots)) + ", Tasks to execute: "
            # + str(len(tasks_to_execute)) + " at " + str(self.env.now))
            # print('FleetManager:     Task list at ' + str(self.env.now) + ': ' + str(
            # [task.to_string() for task in tasks_to_execute]))
            # print('FleetManager:     Robot list at ' + str(self.env.now) + ': ' + str(
            # [robot.to_string() for robot in available_robots]))
    
            # If there are robots available and tasks to execute, optimize and assign
            if not len(tasks_to_execute) == 0 and not len(available_robots) == 0:
    
                # Optimization
                fitness, solution = optimization(len(available_robots), len(tasks_to_execute), cost_matrix)
    
                # Log fitness values for each assignment
                self.fitness_file.write(str(self.env.now) + "," + str(fitness) + ",\n")
    
                # Assign tasks to AGV task lists
                execution_list = dict()
                for i in range(len(available_robots)):
    
                    # Make list of assigned tasks for the robot
                    assigned_tasks = []
                    for j in range(len(tasks_to_execute)):
                        if solution[i][j] == 1:
                            assigned_tasks.append(tasks_to_execute[j])
                            # print("FleetManager:     Assigned task " + str(assigned_tasks[j].order_number)
                            # + " to AGV " + str(available_robots[i].ID) + " at " + str(self.env.now))
    
                    # Pick the task closest to the robot
                    robot_pos = available_robots[i].position
                    ordered_tasks = sorted(assigned_tasks, key=lambda task: self.get_first_task(task, robot_pos))
                    if ordered_tasks:
                        chosen_task = ordered_tasks[0]
                        execution_list[available_robots[i].ID] = chosen_task  # Pick first task (closest)
    
                # Send tasks to AGVs
                for k, v in execution_list.items():
                    self.comm.tcp_write(self.agv_fm_comm[k], v)
            
            else:
                # Log fitness values for no assignment (0.0)
                self.fitness_file.write(str(self.env.now) + "," + str(0.0) + ",\n")

    def define_current_status(self):
    
        # Copy robot list
        robots = np.copy(self.comm.sql_read(self.kb['global_robot_list']))
        robots = sorted(robots, key=lambda robot_: robot_.ID)
    
        # Remove robots which are not idle
        robots_ = []
        for robot in robots:
            if robot.status == 'IDLE':
                robots_.append(robot)
    
        # Copy global task list
        tasks = np.copy(self.comm.sql_read(self.kb['global_task_list']))
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
    
    def get_first_task(self, task, robot_pos):
        distance, _ = self.astar.find_shortest_path(task.pos_A, robot_pos)
        priority = task.priority
        result = distance  # * priority
        return result


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

    # Reformat solution
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
