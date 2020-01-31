import math
import numpy as np
from pyswarm import pso
import matplotlib.pyplot as plt
from TravelingSalesman import TravelingSalesman
import pyomo.environ as pyo
from pyomo.environ import *


class ResourceOptimizer:
    
    def __init__(self, agv):
        self.agv = agv
        self.charging_stations = agv.charging_stations
        self.maximum_resource_time = agv.maximum_resource_time
        self.maximum_resources = agv.maximum_resources
        self.minimum_resources = agv.minimum_resources
        self.resource_scale_factor = agv.resource_scale_factor
        self.initial_resources = None
        self.optimized_tour = None
        self.optimized_resource_time = None
        self.initial_tour = None
        
    def optimize_brute_force(self, initial_tour, initial_resources):
    
        self.initial_resources = initial_resources
        best_fitness = 1000
        best_insertion_point = None
        best_charging_station = None
        best_charging_time = None
        for k in range(len(initial_tour)):
            for j in range(len(self.charging_stations)):
                fitness = self.objective_function_insertion((k, j))
                if fitness < best_fitness:
                    best_fitness = fitness
                    best_insertion_point = k
                    best_charging_station = j
                    best_charging_time = self.optimized_resource_time
                    
        optimal_tour = self.insert_charging_station(best_insertion_point, best_charging_station)
        
        return optimal_tour, best_charging_time, best_fitness
    
    def optimize_nlmip(self):

        # Symbols/expressions
        model = pyo.AbstractModel()
        model.I = pyo.Param(initialize=len(self.initial_tour))
        model.J = pyo.Param(initialize=len(self.charging_stations))
        model.N = pyo.RangeSet(0, model.I)
        model.M = pyo.RangeSet(0, model.J)
        model.x = pyo.Var(model.N, within=PositiveIntegers)
        model.y = pyo.Var(model.M, within=PositiveIntegers)
        model.obj = pyo.Objective(expr=self.objective_function_insertion(model))
        instance = model.create_instance()
        instance.pprint()
        result = SolverFactory('glpk').solve(instance)
        
        return None, None

    def optimize_resource_time(self):
    
        # Bounds
        lb = np.asanyarray([0])
        ub = np.asanyarray([self.maximum_resource_time])
        
        # Objective
        xopt, fopt = pso(self.objective_function_resource_time, lb=lb, ub=ub, ieqcons=[], f_ieqcons=con, args=(),
                         kwargs={}, swarmsize=200, omega=0.5, phip=0.5, phig=0.5, maxiter=100, minstep=1e-4,
                         minfunc=1e-4, debug=False)
    
        return xopt, fopt

    def objective_function_insertion(self, x):
        
        # Convert solution
        charging_position_index, charging_station_index = self.convert_solution(x)
    
        # Insert charging stations
        new_tour = self.insert_charging_station(charging_position_index, charging_station_index)
        self.optimized_tour = new_tour
    
        # Get optimal resource time
        opt_resource_time, fitness = self.optimize_resource_time()
        self.optimized_resource_time = opt_resource_time
        
        return fitness
    
    def objective_function_resource_time(self, x):
    
        # Evaluate tour
        travel_times_between_cities, needed_resource_between_cities, resource_at_city, total_resource_time \
            = self.evaluate_tour(self.optimized_tour, x)

        # Constraints
        punishment = 0
        min_resource = min(resource_at_city)
        max_resource = max(resource_at_city)
        # Resource cannot be less than the minimum resource level
        if min_resource < self.minimum_resources:
            punishment = 1000
        # Resources cannot raise above max resources
        if max_resource > self.maximum_resources:
            punishment = 1000
                
        # Compute total fitness
        total_travel_time = sum(travel_times_between_cities) + total_resource_time
        fitness = total_travel_time + punishment
        return fitness

    def convert_solution(self, x):
        charging_position = x[0]
        charging_station = x[1]
        return charging_position, charging_station

    def insert_charging_station(self, charging_position_index, charging_station_index):
        charging_position = self.initial_tour[charging_position_index]
        charging_station = self.charging_stations[charging_station_index]
        return self.add_charging_station_between_tasks(charging_position, charging_station)
        
    def add_charging_station_between_tasks(self, charging_position, charging_station):
        new_tour = None
        old_tour = self.initial_tour
        for k in range(len(old_tour)):
            task_ = old_tour[k]
            if task_ == charging_position:
                new_tour = old_tour[0:k + 1] + [tuple(np.copy(charging_station))] + old_tour[k + 1:]
                break
        return new_tour

    def evaluate_tour(self, tour_to_evaluate, resource_time=0):
        travel_times_between_cities = self.calculate_travel_times_between_cities(tour_to_evaluate)
        needed_resource_between_cities = self.calculate_needed_resource_between_cities(tour_to_evaluate,
                                                                                       travel_times_between_cities)
        [resource_at_city, total_resource_time] = self.calculate_resource_at_city(
            tour_to_evaluate, needed_resource_between_cities, resource_time)
        return travel_times_between_cities, needed_resource_between_cities, resource_at_city, total_resource_time

    def calculate_travel_times_between_cities(self, tour_to_evaluate):
        travel_times_between_cities = np.zeros(len(tour_to_evaluate) - 1)
        for k in range(len(tour_to_evaluate) - 1):
            city = tour_to_evaluate[k]
            next_city = tour_to_evaluate[k + 1]
            distance = self.agv.astar.find_shortest_path(city.pos_A, next_city.pos_A) # TODO Check this
            delta_t = distance / 1
            travel_times_between_cities[k] = delta_t
        return travel_times_between_cities
    
    def calculate_needed_resource_between_cities(self, tour_to_evaluate, travel_times_between_cities):
        needed_resource_between_cities = np.zeros((len(tour_to_evaluate) - 1))
        for k in range(len(tour_to_evaluate) - 1):
            travel_time_ = travel_times_between_cities[k]
            velocity = 1
            consumed_resources = self.resource_scale_factor * resource_consumption(velocity, travel_time_)
            needed_resource_between_cities[k] = consumed_resources
        return needed_resource_between_cities

    def calculate_resource_at_city(self, tour_to_evaluate, needed_resource_between_cities, resource_time):
        resource_at_city = np.zeros((len(tour_to_evaluate)))
        total_resource_time = 0
        resource_at_city[0] = self.initial_resources
        for k in range(1, len(tour_to_evaluate)):
            if tour_to_evaluate[k] in self.charging_stations:
                total_resource_time = total_resource_time + resource_time
                resource_gradient = self.maximum_resources / self.maximum_resource_time
                added_resource = resource_gradient * resource_time
                resource_at_city[k] = resource_at_city[k - 1] - needed_resource_between_cities[k - 1] + added_resource
            else:
                resource_at_city[k] = resource_at_city[k - 1] - needed_resource_between_cities[k - 1]
    
        return resource_at_city, total_resource_time

    def analysis_resource_time(self):
    
        fitnesses = []
        charging_position_index = 4
        charging_station_index = 1
        new_tour = self.insert_charging_station(charging_position_index, charging_station_index)
        self.optimized_tour = new_tour
        resource_times = np.linspace(0, self.maximum_resource_time, 10)
        for k in range(len(resource_times)):
            fitness = self.objective_function_resource_time(resource_times[k])
            fitnesses.append(fitness)
        plt.figure()
        plt.plot(resource_times, fitnesses)
        plt.show()

    def plot_resource_at_city(self, resource_at_city):
        plt.figure()
        plt.plot(np.arange(len(resource_at_city)), resource_at_city)
        plt.plot(np.arange(len(resource_at_city)), np.repeat(self.minimum_resources, len(resource_at_city)), 'r')
        plt.plot(np.arange(len(resource_at_city)), np.repeat(self.maximum_resources, len(resource_at_city)), 'r')
        plt.show()
        # plt.pause(0.1)
        
    
def calculate_euclidean_distance(a, b):
    return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))


def resource_consumption(_, travel_time_):
    r = 8 * travel_time_
    return r


def plot_tour():

    plt.figure()
    plt.axis('equal')
    for task in locations:
        plt.plot(task[0], task[1], 'r.')
    for station in charge_locations:
        plt.plot(station[0], station[1], 'go')
    plt.plot(robot_location[0], robot_location[1], 'ko')

    for i in range(len(optimized_tour) - 1):
        plt.plot([optimized_tour[i][0], optimized_tour[i + 1][0]], [optimized_tour[i][1], optimized_tour[i + 1][1]],'k')
        plt.plot([optimized_tour[-1][0], optimized_tour[0][0]], [optimized_tour[-1][1], optimized_tour[0][1]], 'k')
    plt.show()
    
    
def con(x):
    return 0
    

if __name__ == '__main__':
    
    # Tasks
    locations = [(0, 0), (0, 24), (0, 45), (24, 36), (24, 0), (18, 0), (15, 12), (9, 12), (3, 0)]
    
    # Tasks
    print("Tasks set")
    print(['(' + str(task[0]) + ', ' + str(task[1]) + ')' for task in locations])
    
    # Robot location
    robot_location = (0, 0)
    
    # Charging locations
    charge_locations = [(9, 21), (12, 30), (3, 36)]
    
    # Optimized tour
    tsp = TravelingSalesman(None)
    optimized_tour, travel_time = tsp.compute_tsp(locations, robot_location)
    
    # Plot optimized tour
    print("\nOptimized tour")
    print(['(' + str(task[0]) + ', ' + str(task[1]) + ')' for task in optimized_tour])
    print("Total traveltime: " + str(travel_time))

    # Define MIP Optimizer
    # ropt = ResourceOptimizer(None)
    # ropt.charge_locations = [(9, 21), (12, 30), (3, 36)]
    # ropt.initial_tour = optimized_tour
    
    # Evaluate optimized tour
    # _, _, res_at_cit, _ = mip.evaluate_tour(optimized_tour, 0)
    # mip.plot_resource_at_city(res_at_cit)
    
    # Check different initial resources
    # plt.figure()
    # initial_resources = np.linspace(100, 0, 10)
    # for i in range(0): #len(initial_resources)):
        # mip.initial_resources = initial_resources[i]
        # Check if charging is needed
        # _, _, resource_at_city_, _ = mip.evaluate_tour(optimized_tour, 0)
    
        # If no charging needed, return optimized tour, otherwise insert charging station
        # optimized_resource_time = 0
        # if resource_at_city_[-1] > mip.min_resources:
            # print("\nNo need to optimize")
            # optimized_tour_final = optimized_tour
        # else:
            # print("\nNeed to optimize")
            # optimized_tour_final, optimized_resource_time = mip.optimize()
            
        # print('Final tour: ', optimized_tour_final)
        # print('Final resource time: ', optimized_resource_time)
        # travel_times_, _, resource_at_city_, total_resource_time_ = \
            # mip.evaluate_tour(optimized_tour_final, optimized_resource_time)
        # mip.plot_resource_at_city(resource_at_city_)
        # total_travel_time_ = sum(travel_times_) + total_resource_time_
        # print("Total travel time: " + str(total_travel_time_))

    # optimized_tour_final, optimized_resource_time = ropt.optimize_nlmip()
    
    #_, _, rec_at_cit, _ = mip.evaluate_tour(optimized_tour_final, optimized_resource_time)
    #mip.plot_resource_at_city(rec_at_cit)