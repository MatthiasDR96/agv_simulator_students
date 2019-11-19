import ast
import configparser
import random

import matplotlib.pyplot as plt
import simpy

from AGV import AGV
from Astar import Astar
from FleetManager import FleetManager
from Logger import Logger
from MES import MES
from Node import Node


class Simulation:

    def __init__(self):

        # Setup
        setup = configparser.ConfigParser()
        setup.read('test_vectors/setup.ini')

        # Set params
        self.number_of_agvs = int(setup['GENERAL']['number_of_agvs'])  # Number of AGVs in the system
        self.robot_speed = int(setup['GENERAL']['robot_speed'])  # Robot speed
        self.task_execution_time = int(setup['GENERAL']['task_execution_time'])  # Execution time of tasks
        self.node_locations = ast.literal_eval(setup['LAYOUT']['node_locations'])  # List of locations of nodes
        self.node_neighbors = ast.literal_eval(setup['LAYOUT']['node_neighbors'])  # List of edges between nodes
        self.node_names = ast.literal_eval(setup['LAYOUT']['node_names'])  # List of node names
        self.charge_locations = ast.literal_eval(setup['LAYOUT']['charge_locations'])  # List of charging locations
        self.depot_locations = ast.literal_eval(setup['LAYOUT']['depot_locations'])  # List of depot locations
        self.order_list = str(setup['ORDERS']['order_list'])  # List of orders for MES to execute

        # Start simulation
        res = self.start_simulation()

        # Print simulation results
        duration = open("logfiles/simulation_duration.txt", "w")
        duration.write(str(res))
        duration.close()
        print("Simulation time: " + str(res))

    def print_simulation_info(self):

        print("\nSimulation started\n")
        print("Amount of AGVs: " + str(self.number_of_agvs))
        print("All AGVs drive at a constant speed of: " + str(self.robot_speed) + " m/s.")
        print("AGVs start locations are random chosen.")
        print("Fleet manager uses a simple random optimizer.")
        print("AGVs pick the first task from their local taks list and execute the task completely (A and B) before"
              " starting a new task.")

    def start_simulation(self):

        # Print info
        self.print_simulation_info()

        # Construct graph
        graph = self.make_graph()

        # Init Astar
        astar = Astar(graph)

        # Define simulation environment
        env = simpy.Environment()

        # Define global task list, global robot list, and list of tasks executing
        global_task_list = simpy.FilterStore(env)
        global_robot_list = simpy.FilterStore(env)
        tasks_executing = simpy.FilterStore(env)

        # Define local_task list for each AGV
        local_task_lists = [simpy.FilterStore(env) for i in range(self.number_of_agvs)]

        # Define MES
        mes = MES(env, global_task_list, tasks_executing, self.order_list)

        # Define Fleet Manger
        FleetManager(env, global_task_list, global_robot_list, local_task_lists, astar)

        # Define AGVs starting at a random location
        for ID in range(self.number_of_agvs):
            AGV(env, ID + 1, self.robot_speed, global_task_list, global_robot_list, random.choice(self.depot_locations),
                local_task_lists[ID], astar, self.task_execution_time, self.charge_locations, tasks_executing)

        # Define renderer
        # Renderer(env, graph, global_task_list, global_robot_list, local_task_lists, tasks_executing,
        # self.depot_locations, self.charge_locations)

        # Define logger
        Logger(env, global_task_list, global_robot_list, local_task_lists, tasks_executing)

        # Run environment
        env.run(until=mes.main)

        # Return duration of simulation
        return env.now

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


def print_layout(locations):
    print("AGV layout printed:\n")
    for node in locations:
        print("\t" + node.to_string())
        print("\t\tNeighbors:")
        print("\t\t" + str([neighbor.to_string() for neighbor in node.neighbors]))


def plot_layout(locations):
    plt.figure(1)
    for node in locations:
        plt.plot(node.pos[0], node.pos[1], 'b.', ms=6)
        plt.text(node.pos[0], node.pos[1] + 0.1, node.name)
        for neighbor in node.neighbors:
            plt.plot([node.pos[0], neighbor.pos[0]], [node.pos[1], neighbor.pos[1]], 'b-', lw=0.5)
    plt.show()


if __name__ == '__main__':
    # Run simulation
    sim = Simulation()
