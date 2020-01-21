import ast
import configparser

import matplotlib.pyplot as plt
import simpy

from AGV import AGV
from FleetManager import FleetManager
from MES import MES
from Node import Node


class Simulation:
    """
            This is the baseclass for the simulation, the whole simulation is set up and started from here.
    """
    
    def __init__(self):
        
        # Setup
        setup = configparser.ConfigParser()
        setup.read('../test_vectors/setup.ini')
        
        # Set params
        self.number_of_agvs = int(setup['GENERAL']['number_of_agvs'])  # Number of AGVs in the system
        self.robot_speed = int(setup['GENERAL']['robot_speed'])  # Robot speed
        self.task_execution_time = int(setup['GENERAL']['task_execution_time'])  # Execution time of tasks
        self.battery_threshold = int(setup['GENERAL']['battery_threshold'])
        self.max_charging_time = int(setup['GENERAL']['max_charging_time'])
        self.node_locations = ast.literal_eval(setup['LAYOUT']['node_locations'])  # List of locations of nodes
        self.node_neighbors = ast.literal_eval(setup['LAYOUT']['node_neighbors'])  # List of edges between nodes
        self.node_names = ast.literal_eval(setup['LAYOUT']['node_names'])  # List of node names
        self.charge_locations = ast.literal_eval(setup['LAYOUT']['charge_locations'])  # List of charging locations
        self.depot_locations = ast.literal_eval(setup['LAYOUT']['depot_locations'])  # List of depot locations
        self.start_locations = ast.literal_eval(setup['LAYOUT']['start_locations'])  # List of starting locations
        self.order_list = str(setup['ORDERS']['order_list'])  # List of orders for MES to execute
        
        # Check correctness of setup
        if not self.number_of_agvs <= len(self.start_locations):
            print("Simulation setup not correct")
            exit()
        else:
            # Start simulation
            res = self.start_simulation()

            # Print simulation results
            with open("../logfiles/simulation_duration.txt", "w") as duration:
                duration.write(str(res))
            print("Simulation time: " + str(res))
    
    def start_simulation(self):
        
        # Print info
        self.print_simulation_info()
        
        # Define simulation environment
        env = simpy.Environment()
        
        # Define knowledge base
        kb = self.define_knowledge_base(env)
        
        # Define communication channel between FleetManager and AGVs
        agv_fm_comm = dict()
        [agv_fm_comm.update({i + 1: simpy.FilterStore(env)}) for i in range(self.number_of_agvs)]
        
        # Define MES
        mes = MES(env, kb, self.order_list)
        
        # Define Fleet Manger
        FleetManager(env, kb, agv_fm_comm)
        
        # Define AGVs
        for ID in range(self.number_of_agvs):
            agv_params = {'ID': ID + 1,
                          'robot_speed': self.robot_speed,
                          'task_execution_time': self.task_execution_time,
                          'start_location': self.start_locations[ID],
                          'battery_threshold': self.battery_threshold,
                          'max_charging_time': self.max_charging_time}
            AGV(env, agv_params, kb, agv_fm_comm[ID + 1])
        
        # Define logger
        # Logger(env, kb)
        
        # Define online renderer
        # RendererOnline(env, kb, self.depot_locations, self.charge_locations)
        
        # Run environment
        env.run(until=mes.main)
        
        # Return duration of simulation
        return env.now
    
    def print_simulation_info(self):
        
        print("\nSimulation started\n")
        print("Amount of AGVs: " + str(self.number_of_agvs))
        print("All AGVs drive at a constant speed of: " + str(self.robot_speed) + " m/s.")
        print("AGVs start locations are fixed.")
        print("Fleet manager uses a linear programming optimizer.")
    
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
    
    def define_knowledge_base(self, env):
        global_task_list = simpy.FilterStore(env)
        global_robot_list = simpy.FilterStore(env)
        tasks_executing = simpy.FilterStore(env)
        graph = self.make_graph()
        kb = dict()
        kb['global_task_list'] = global_task_list
        kb['global_robot_list'] = global_robot_list
        kb['tasks_executing'] = tasks_executing
        kb['charge_locations'] = self.charge_locations
        kb['graph'] = graph
        return kb


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
