import simpy
from MES import MES
from FleetManager import FleetManager
from Renderer import Renderer
from AGV import AGV
from random import randint
from Node import Node
from Astar import Astar


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


def print_locations(locations):
    print("AGV layout printed:\n")
    for node in locations:
        print("\t" + node.to_string())
        print("\t\tNeighbors:")
        print("\t\t" + str([neighbor.to_string() for neighbor in node.neighbors]))


if __name__ == "__main__":

    # Init
    number_of_agvs = 3
    robot_speed = 1.0  # m/s
    collision_threshold = 0.5  # AGV diameter is 1m
    amount_of_tasks = 10
    max_arrival_interval = 20

    # Define graph
    task_locations = [(10, 20), (10, 40), (10, 60), (20, 0), (20, 20), (20, 40), (20, 60), (20, 80), (30, 0), (30, 20), (30, 40), (30, 60), (30, 80)]
    node_locations = [(10, 20), (10, 40), (10, 60), (20, 0), (20, 20), (20, 40), (20, 60), (20, 80), (30, 0), (30, 20), (30, 40), (30, 60), (30, 80)]
    node_names = ["pos_1", "pos_2", "pos_3", "pos_4", "pos_5", "pos_6", "pos_7", "pos_8", "pos_9", "pos_10", "pos_11", "pos_12", "pos_13"]
    node_neighbors = [["pos_5"], ["pos_6"], ["pos_7"], ["pos_5", "pos_9"], ["pos_1", "pos_4", "pos_6"],
                    ["pos_2", "pos_5", "pos_7", "pos_10", "pos_11", "pos_12"],
                    ["pos_3", "pos_6", "pos_8"], ["pos_7", "pos_13"], ["pos_4", "pos_10"], ["pos_6", "pos_9", "pos_11"],
                    ["pos_6", "pos_10", "pos_12"], ["pos_6", "pos_11", "pos_13"], ["pos_8", "pos_12"]]

    # Print information
    print("\nSimulation started\n")
    print("Amount of AGVs: " + str(number_of_agvs))
    print("All AGVs drive at a constant speed of: " + str(robot_speed) + " m/s.")
    print("AGVs start locations are random chosen.")
    print("Fleet manager uses a simple random optimizer.\n")

    # Construct graph
    graph = make_graph()
    print_locations(graph)

    # Init Astar
    astar = Astar(graph)

    # Define non-realtime environment
    env = simpy.Environment()

    ####################################################################################################

    # Define global task list and global robot list
    global_task_list = simpy.FilterStore(env)
    global_robot_list = simpy.FilterStore(env)

    # Define local_task list for each AGV
    local_task_lists = [simpy.FilterStore(env) for i in range(number_of_agvs)]

    ####################################################################################################

    # Define MES
    mes = MES(env, global_task_list, task_locations, amount_of_tasks, max_arrival_interval)

    # Define Fleet Manger
    fleet_manager = FleetManager(env, global_task_list, global_robot_list, local_task_lists, astar)

    # Define AGVs starting at a random location
    AGVs = [AGV(env, ID+1, robot_speed, global_task_list, global_robot_list, node_locations[randint(0, len(node_locations))],
                local_task_lists[ID], astar) for ID in range(number_of_agvs)]

    # Define renderer
    #renderer = Renderer(env, graph, global_task_list, global_robot_list)

    #####################################################################################################

    # Run environment
    env.run()

