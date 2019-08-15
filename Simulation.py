from random import randint

import matplotlib.pyplot as plt
import numpy as np
import simpy

from AGV import AGV
from Astar import Astar
from FleetManager import FleetManager
from MES import MES
from Node import Node
from Renderer import Renderer


def make_graph():
    # Make nodes
    nodes = []
    for j in range(len(node_locations)):
        node = Node()
        node.pos = node_locations[j]
        node.name = node_names[j]
        node.neighbors = node_neighbors[j]
        nodes.append(node)

    # Replace neighbor names by node objects
    for node in nodes:
        for j in range(len(node.neighbors)):
            for k in range(len(node_names)):
                if node_names[k] == node.neighbors[j]:
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
    plt.draw()
    plt.pause(1000)


if __name__ == "__main__":

    # Parameter initialization
    number_of_agvs = 3
    robot_speed = 1.0  # m/s
    collision_threshold = 0.5  # AGV diameter is 1m
    amount_of_tasks_to_execute = 10
    max_arrival_interval = 20  # sec
    task_execution_time = 2  # sec

    # Define graph
    node_locations = [(10, 20), (10, 40), (10, 60), (20, 0), (20, 20), (20, 40), (20, 60), (20, 80), (30, 0), (30, 20),
                      (30, 40), (30, 60), (30, 80)]
    task_locations = np.delete(node_locations, 1, axis=0)  # No tasks at depot location and charging locations
    depot_locations = node_locations[1]
    charge_locations = [node_locations[1], node_locations[9]]
    node_names = ["pos_1", "pos_2", "pos_3", "pos_4", "pos_5", "pos_6", "pos_7", "pos_8", "pos_9", "pos_10", "pos_11",
                  "pos_12", "pos_13"]
    node_neighbors = [["pos_5"], ["pos_6"], ["pos_7"], ["pos_5", "pos_9"], ["pos_1", "pos_4", "pos_6"],
                      ["pos_2", "pos_5", "pos_7", "pos_10", "pos_11", "pos_12"],
                      ["pos_3", "pos_6", "pos_8"], ["pos_7", "pos_13"], ["pos_4", "pos_10"],
                      ["pos_6", "pos_9", "pos_11"],
                      ["pos_6", "pos_10", "pos_12"], ["pos_6", "pos_11", "pos_13"], ["pos_8", "pos_12"]]

    # Print information
    print("\nSimulation started\n")
    print("Amount of AGVs: " + str(number_of_agvs))
    print("All AGVs drive at a constant speed of: " + str(robot_speed) + " m/s.")
    print("AGVs start locations are random chosen.")
    print("Fleet manager uses a simple random optimizer.")
    print("AGVs pick the first task from their local taks list and execute the task completely (A and B) before"
          " starting a new task.\n")

    # Construct graph
    graph = make_graph()
    # print_layout(graph)
    # plot_layout(graph)

    # Init Astar
    astar = Astar(graph)

    # Define non-realtime environment
    env = simpy.Environment()

    ####################################################################################################

    # Define global task list and global robot list
    global_task_list = simpy.FilterStore(env)
    global_robot_list = simpy.FilterStore(env)
    tasks_executing = simpy.FilterStore(env)

    # Define local_task list for each AGV
    local_task_lists = [simpy.FilterStore(env) for i in range(number_of_agvs)]

    # To overcome strange error
    for ID in range(number_of_agvs):
        continue

    ####################################################################################################

    # Define MES
    mes = MES(env, global_task_list, task_locations, amount_of_tasks_to_execute, max_arrival_interval, tasks_executing)

    # Define Fleet Manger
    fleet_manager = FleetManager(env, global_task_list, global_robot_list, local_task_lists, astar)

    # Define AGVs starting at a random location
    AGVs = [AGV(env, ID + 1, robot_speed, global_task_list, global_robot_list,
                node_locations[randint(0, len(node_locations))],
                local_task_lists[ID], astar, task_execution_time, charge_locations, tasks_executing) for ID in
            range(number_of_agvs)]

    # Define renderer
    renderer = Renderer(env, graph, global_task_list, global_robot_list, local_task_lists, tasks_executing,
                        depot_locations, charge_locations)

    #####################################################################################################

    # Run environment
    env.run(until=mes.main)
