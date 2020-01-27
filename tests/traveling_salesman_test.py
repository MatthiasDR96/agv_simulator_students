from __future__ import print_function

import ast
import configparser

import matplotlib.pyplot as plt
import numpy as np
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

from Astar import Astar
from Node import Node
from Task import Task


def plot_layout(locations, plan):
    plt.figure(1)
    i = 0
    for node in locations:
        plt.plot(node[0], node[1], 'b.', ms=6)
        plt.text(node[0], node[1] + 0.1, str(i))
        i += 1
    for i in range(len(plan) - 1):
        index = plan[i]
        indexx = plan[i + 1]
        plt.plot([locations[index][0], locations[indexx][0]], [locations[index][1], locations[indexx][1]], 'b-', lw=0.5)
    plt.show()


def get_plan(manager, routing, assignment):
    index = routing.Start(0)
    plan_output = []
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output.append(manager.IndexToNode(index))
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    return route_distance, plan_output


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


def distance_callback(from_index, to_index):
    """Returns the distance between the two nodes."""
    # Convert from routing variable Index to distance matrix NodeIndex.
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return data['distance_matrix'][from_node][to_node]


if __name__ == "__main__":
    
    # Setup
    setup = configparser.ConfigParser()
    setup.read('../test_vectors/setup.ini')
    
    # Open file
    order_list = str(setup['ORDERS']['order_list'])
    node_locations = ast.literal_eval(setup['LAYOUT']['node_locations'])  # List of locations of nodes
    node_neighbors = ast.literal_eval(setup['LAYOUT']['node_neighbors'])  # List of edges between nodes
    node_names = ast.literal_eval(setup['LAYOUT']['node_names'])  # List of node names
    file = open(order_list, "r")
    
    # Read order
    tasks = []
    line = file.readline()
    order = line.split(',')
    prev_time = 0
    while line != "":
        # Create new task
        order_number = int(order[1])
        pos_a = (int(order[3]), int(order[4]))
        pos_b = (int(order[5]), int(order[6]))
        priority = int(order[2])
        new_task = Task(order_number, pos_a, pos_b, priority)
        tasks.append(new_task)
        
        # Read order
        line = file.readline()
        order = line.split(',')
    
    # Close file
    file.close()
    
    graph = make_graph()
    astar = Astar(graph)  # Astar shortest path finder
    
    locations = []
    for task in tasks:
        locations.append(task.pos_A)
        locations.append(task.pos_B)
    locations = [(10, 40), (10, 20), (30, 20), (30, 80), (30, 60), (20, 80), (20, 40), (30, 0)]
    
    print("Locations")
    print(locations)
    print("")
    
    data = dict()
    data['distance_matrix'] = np.zeros((len(locations), len(locations)))
    for i in range(len(locations)):
        for j in range(len(locations)):
            distance, _ = astar.find_shortest_path(locations[i], locations[j])
            data['distance_matrix'][i, j] = distance
    data['num_vehicles'] = 1
    data['depot'] = 0
    
    print("Cost matrix")
    print(data)
    
    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    
    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    
    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    
    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    
    # Print solution on console.
    distance, plan = get_plan(manager, routing, assignment)
    print(plan)
    
    plot_layout(locations, plan)
    
    optimized_tour = []
    for i in plan:
        optimized_tour.append(locations[i])
    
    print(optimized_tour)
