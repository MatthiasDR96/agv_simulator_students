from __future__ import print_function

import numpy as np
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


class TravelingSalesman:
    """
            A class containing the Traveling Salesman Optimization
    """
    
    def __init__(self, astar):
        self.astar = astar
        self.data = dict()
        self.manager = None
    
    def compute_tsp(self, tasks, robot_location):
        
        # Compute cost matrix
        self.data = self.create_data(tasks, robot_location)
        
        # Create the routing index manager.
        self.manager = pywrapcp.RoutingIndexManager(len(self.data['distance_matrix']), self.data['num_vehicles'],
                                                    self.data['depot'])
        
        # Create Routing Model.
        routing = pywrapcp.RoutingModel(self.manager)
        
        transit_callback_index = routing.RegisterTransitCallback(self.distance_callback)
        
        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
        
        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        
        # Solve the problem.
        assignment = routing.SolveWithParameters(search_parameters)
        
        # Print solution on console.
        distance, plan = get_plan(self.manager, routing, assignment)
        
        optimized_tour = []
        for i in plan:
            optimized_tour.append(tasks[i])
        
        distance, _ = self.astar.find_shortest_path(robot_location, optimized_tour[0].pos_A)
        for i in range(len(optimized_tour) - 1):
            distance_1, _ = self.astar.find_shortest_path(optimized_tour[i].pos_A, optimized_tour[i].pos_B)
            distance_2, _ = self.astar.find_shortest_path(optimized_tour[i].pos_B, optimized_tour[i + 1].pos_A)
            distance += distance_1 + distance_2
        
        return distance, optimized_tour
    
    def distance_callback(self, from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        return self.data['distance_matrix'][from_node][to_node]
    
    def create_data(self, tasks, robot_location):
        data = dict()
        data['distance_matrix'] = np.zeros((len(tasks), len(tasks)))
        for i in range(len(tasks)):
            for j in range(len(tasks)):
                if i == j:
                    data['distance_matrix'][i, j] = 0
                else:
                    distance_r1, _ = self.astar.find_shortest_path(robot_location, tasks[i].pos_A)
                    distance_12, _ = self.astar.find_shortest_path(tasks[i].pos_B, tasks[j].pos_A)
                    data['distance_matrix'][i, j] = distance_r1 + distance_12
        data['num_vehicles'] = 1
        data['depot'] = 0
        return data


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


def get_locations(tasks, robot_location):
    locations = [robot_location]
    for task in tasks:
        locations.append(task.pos_A)
        locations.append(task.pos_B)
    return locations
