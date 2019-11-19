import ast
import configparser
import math

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from Node import Node

plt.ion()


class RendererOffLine:

    def __init__(self):

        # Log files
        self.global_robot_list_logfile = open("../logfiles/global_robot_list.txt", "r")
        self.global_task_list_logfile = open("../logfiles/global_task_list.txt", "r")
        self.local_task_list_logfile = open("../logfiles/local_task_lists.txt", "r")
        self.tasks_executing_logfile = open("../logfiles/tasks_executing.txt", "r")

        # Setup file
        setup = configparser.ConfigParser()
        setup.read('../test_vectors/setup.ini')

        # Attributes
        self.global_robot_list = self.global_robot_list_logfile
        self.global_task_list = self.global_task_list_logfile
        self.local_task_lists = self.local_task_list_logfile
        self.tasks_executing = self.tasks_executing_logfile
        duration = open("../logfiles/simulation_duration.txt", "r")
        self.simulation_duration = int(duration.readline())
        duration.close()
        self.number_of_agvs = int(setup['GENERAL']['number_of_agvs'])  # Number of AGVs in the system
        self.colors = matplotlib.cm.rainbow(np.linspace(0, 1, self.number_of_agvs))
        self.node_locations = ast.literal_eval(setup['LAYOUT']['node_locations'])  # List of locations of nodes
        self.node_neighbors = ast.literal_eval(setup['LAYOUT']['node_neighbors'])  # List of edges between nodes
        self.charge_locations = ast.literal_eval(setup['LAYOUT']['charge_locations'])  # List of charging locations
        self.depot_locations = ast.literal_eval(setup['LAYOUT']['depot_locations'])  # List of charging locations
        self.node_names = ast.literal_eval(setup['LAYOUT']['node_names'])  # List of node names
        self.graph = self.make_graph()
        self.time = 0
        self.time_axis = []
        self.status_axis = [[] for i in range(self.number_of_agvs)]
        self.battery_axis = [[] for i in range(self.number_of_agvs)]

        # Initiate plt
        plt.close("all")
        plt.rcParams['toolbar'] = 'None'
        plt.figure("AGV Simulator", figsize=(11, 6), facecolor='k', edgecolor='k', dpi=100)
        plt.title("AGV Simulator")

        # Start offline rendering
        self.render_scene()

        # Close files
        self.global_robot_list_logfile.close()
        self.global_task_list_logfile.close()
        self.local_task_list_logfile.close()
        self.tasks_executing_logfile.close()

    def render_scene(self):

        for i in range(self.simulation_duration):
            # Read status at time i
            global_robot_list = self.global_robot_list_logfile.readline()
            self.time = int(global_robot_list.split('/')[0])
            self.time_axis.append(self.time)
            self.global_robot_list = global_robot_list.split('/')[1].split(':')
            global_task_list = self.global_task_list_logfile.readline()
            self.global_task_list = global_task_list.split('/')[1].split(':')
            local_task_lists = self.local_task_list_logfile.readline()
            self.local_task_lists = local_task_lists.split('/')[1].split(':')
            tasks_executing = self.tasks_executing_logfile.readline()
            self.tasks_executing = tasks_executing.split('/')[1].split(':')

            # Configure subplots
            plt.subplot2grid((1, self.number_of_agvs + 3), (0, 0), colspan=3)
            plt.subplots_adjust(wspace=1.5, hspace=0.4)
            plt.title("AGV Layout")
            plt.style.use('dark_background')

            # Plot graph
            self.plot_graph()

            # Plot AGVs
            self.plot_AGVs()

            # Plot global tasks
            self.plot_global_tasks()

            # Plot executing tasks
            self.plot_executing_tasks()

            # Plot status
            self.plot_status()

            # Plot battery
            self.plot_battery()

            # Plot
            plt.draw()
            plt.pause(0.0001)
            plt.clf()
            plt.tight_layout()

            # Sleep
            # time.sleep(0.05)

    def plot_status(self):
        global_robot_list = list(self.global_robot_list)
        for i in range(self.number_of_agvs):
            robot = global_robot_list[i].split(';')
            plt.subplot2grid((2, self.number_of_agvs + 3), (0, int(robot[0]) + 2))
            plt.title("AGV" + str(int(robot[0])))
            plt.xlabel("Simulation time (s)")
            plt.ylabel("IDLE (0), BUSY (1)")
            if robot[3] == 'BUSY':
                state = 1
            else:
                state = 0
            self.status_axis[i].append(state)
            plt.plot(self.time_axis, self.status_axis[i], 'b')

    def plot_battery(self):
        global_robot_list = list(self.global_robot_list)
        for i in range(self.number_of_agvs):
            robot = global_robot_list[i].split(';')
            plt.subplot2grid((2, self.number_of_agvs + 3), (1, int(robot[0]) + 2))
            plt.xlabel("Simulation time (s)")
            plt.ylabel("Battery level")
            plt.ylim(0, 100)
            battery_status = float(robot[4])
            self.battery_axis[i].append(battery_status)
            plt.plot(self.time_axis, self.battery_axis[i], 'b')
            plt.plot(self.time_axis, np.repeat(20, len(self.battery_axis[i])), 'r')

    def plot_global_tasks(self):
        # Plot tasks not executing
        global_task_list = list(self.global_task_list)
        for i in range(len(global_task_list) - 1):
            task = global_task_list[i].split(';')
            if not task == "":
                plt.plot(int(task[1]), int(task[2]), 'gs', ms=7)
                plt.text(int(task[1]) - 1, int(task[2]) + 1.5, str((task[0])))

    def plot_executing_tasks(self):
        # Plot tasks executing
        tasks_executing = list(self.tasks_executing)
        global_robot_list = list(self.global_robot_list)
        for i in range(len(tasks_executing) - 1):
            task = tasks_executing[i].split(';')
            if not int(task[-1]):
                plt.plot(int(task[1]), int(task[2]), 'gs', ms=7)
                plt.text(int(task[1]) - 1, int(task[2]) + 1.5, str(task[0]))
            else:
                for i in range(len(global_robot_list) - 1):
                    robot = global_robot_list[i].split(';')
                    if int(task[-3][1]) == int(robot[0]):
                        plt.plot(float(robot[1]), float(robot[2]), 'gs', ms=7)
                        plt.text(float(robot[1]) - 1, float(robot[2]) + 1.5, str(task[0]))

    def plot_AGVs(self):

        # Plot AGVs
        global_robot_list = list(self.global_robot_list)
        local_task_lists = list(self.local_task_lists)
        for i in range(len(global_robot_list) - 1):
            robot = global_robot_list[i].split(';')
            color = self.colors[int(robot[0]) - 1]
            plt.plot(float(robot[1]), float(robot[2]), color=color, marker='o', ms=10)
            plt.arrow(float(robot[1]), float(robot[2]), math.cos(float(robot[5])),
                      math.sin(float(robot[5])), width=0.3, color=color)
            plt.text(float(robot[1]) + 0.5, float(robot[2]) + 1.5, str("AGV" + str(robot[0])))

            # Plot path
            path = robot[6]
            if not path == "":
                path = path.split(',')[:-1]
                plt.plot([float(robot[1]), int(path[0])], [float(robot[2]), int(path[1])],
                         color=color, lw=1.0)
                for k in range(0, len(path), 2):
                    plt.plot(int(path[k]), int(path[k + 1]), color=color, marker='.', ms=10)
                    if k < len(path) - 2:
                        plt.plot([int(path[k]), int(path[k + 2])],
                                 [int(path[k + 1]), int(path[k + 3])], color=color, lw=1.5)

            # Plot assigned tasks
            list_ = local_task_lists[int(robot[0]) - 1].split(',')[1]
            if not list_ == "[]":
                list_ = list_[2:-2].split(', ')
                for i in range(len(list_)):
                    task = list_[i].split(';')
                    plt.plot([float(robot[1]), int(task[1])],
                             [float(robot[2]), int(task[2])], color=color, lw=0.5)

    def plot_graph(self):

        for node in self.graph:
            if node.pos in self.depot_locations:
                plt.plot(node.pos[0], node.pos[1], 'g^', ms=6)
            elif node.pos in self.charge_locations:
                plt.plot(node.pos[0], node.pos[1], 'g^', ms=6)
            else:
                plt.plot(node.pos[0], node.pos[1], 'b.', ms=6)
                # plt.text(node.pos[0]-1, node.pos[1]+1, node.name)
                for neighbor in node.neighbors:
                    plt.plot([node.pos[0], neighbor.pos[0]], [node.pos[1], neighbor.pos[1]], 'b-', lw=0.5)

    def make_graph(self):

        # Make nodes
        nodes = []
        for j in range(len(self.node_locations)):
            node = Node()
            node.pos = self.node_locations[j]
            node.name = self.node_names[j]
            node.neighbors = self.node_neighbors[j]
            nodes.append(node)

        # Replace neighbor names by node objects
        for node in nodes:
            for j in range(len(node.neighbors)):
                for k in range(len(self.node_names)):
                    if self.node_names[k] == node.neighbors[j]:
                        node.neighbors[j] = nodes[k].copy_node()

        return nodes


if __name__ == "__main__":
    # Start rendering
    render = RendererOffLine()
