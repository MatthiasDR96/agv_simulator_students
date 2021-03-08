import numpy as np

from src.solvers.astar_solver import *


def calculate_euclidean_distance(a, b):
    return math.sqrt(math.pow(b[0] - a[0], 2) + math.pow(b[1] - a[1], 2))


def calculate_astar_distance(graph, a, b):
    _, distance = find_shortest_path(graph, a, b)
    return distance


def calculate_path_distance(graph, path):
    path_distance = 0
    for i in range(len(path) - 1):
        distance = graph.edges[path[i], path[i + 1]].length
        path_distance += distance
    return path_distance


def calculate_path_traveltime(graph, path, speed):
    path_travel_time = 0
    for i in range(len(path) - 1):
        distance = graph.edges[path[i], path[i + 1]].length
        travel_time = distance / speed
        path_travel_time += travel_time
    return path_travel_time


def get_travel_cost_per_robot():
    f = open("../logfiles/global_robot_list.txt", "r")
    lines = f.readlines()
    last_line = lines[-1]
    global_robot_list = last_line.split('|')[1:-1]
    travel_cost_per_robot = np.zeros(len(global_robot_list))
    for i in range(len(global_robot_list)):
        robot = global_robot_list[i].split(';')
        cost = float(robot[7])
        travel_cost_per_robot[i] = cost
    f.close()
    return travel_cost_per_robot


def get_charging_cost_per_robot():
    f = open("../logfiles/global_robot_list.txt", "r")
    lines = f.readlines()
    last_line = lines[-1]
    global_robot_list = last_line.split('|')[1:-1]
    charging_cost_per_robot = np.zeros(len(global_robot_list))
    for i in range(len(global_robot_list)):
        robot = global_robot_list[i].split(';')
        cost = float(robot[8])
        charging_cost_per_robot[i] = cost
    f.close()
    return charging_cost_per_robot


def get_congestions_per_robot():
    f = open("../logfiles/global_robot_list.txt", "r")
    lines = f.readlines()
    last_line = lines[-1]
    global_robot_list = last_line.split('|')[1:-1]
    congestions_per_robot = np.zeros(len(global_robot_list))
    for i in range(len(global_robot_list)):
        robot = global_robot_list[i].split(';')
        cost = float(robot[9])
        congestions_per_robot[i] = cost
    f.close()
    return congestions_per_robot
