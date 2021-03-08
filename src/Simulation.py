import simpy

from src.agv.AGV_Main import AGV
from src.Logger import Logger
from src.MES import MES
from src.Graph import Graph
from src.RendererOnline import RendererOnline
from src.fleetmanagers.FleetManager import FleetManager
from src.utils.situation_generator import *
from src.utils.utils import *


class Simulation:

    """
            This is the baseclass for the simulation, the whole simulation is set up and started from here.
    """
    
    def __init__(self, setup_file_path):
        
        # Setup
        self.setup_file_path = setup_file_path
        self.setup = configparser.ConfigParser()
        self.setup.read(setup_file_path)

        # Set general params
        self.robot_speed = float(self.setup['GENERAL']['robot_speed'])  # Robot speed
        self.task_execution_time = float(self.setup['GENERAL']['task_execution_time'])  # Execution time of tasks
        self.battery_threshold = float(self.setup['GENERAL']['battery_threshold'])
        self.collision_threshold = float(self.setup['GENERAL']['collision_threshold'])
        self.max_charging_time = float(self.setup['GENERAL']['max_charging_time'])
        self.max_tasks_in_task_list = float(self.setup['GENERAL']['max_tasks_in_task_list'])
        self.epsilon = float(self.setup['GENERAL']['epsilon'])  # Objective parameter
        self.initial_resources = float(self.setup['GENERAL']['initial_resources'])

        # Set layout params
        self.node_locations = ast.literal_eval(self.setup['LAYOUT']['node_locations'])  # List of locations of nodes
        self.node_neighbors = ast.literal_eval(self.setup['LAYOUT']['node_neighbors'])  # List of edges between nodes
        self.node_names = ast.literal_eval(self.setup['LAYOUT']['node_names'])  # List of node names
        self.charge_locations = ast.literal_eval(self.setup['LAYOUT']['charge_locations'])  # List of charging locations
        self.depot_locations = ast.literal_eval(self.setup['LAYOUT']['depot_locations'])  # List of depot locations
        self.start_locations = ast.literal_eval(self.setup['LAYOUT']['start_locations'])  # List of starting locations
        self.task_locations = ast.literal_eval(self.setup['LAYOUT']['task_locations'])  # List of task locations

        # Create layout
        self.graph = Graph()
        self.graph.create_nodes(self.node_locations, self.node_names)
        self.graph.create_edges(self.node_names, self.node_neighbors)
    
    def start_simulation(self, order_list, num_robots, render_=True, log_=True, print_=True):
        
        # Define simulation environment
        env = simpy.Environment()
        
        # Define knowledge base (SQL database)
        kb = self.define_knowledge_base(env)
        
        # Define communication channel between FleetManager and AGVs (These are just the virtual IP adresses)
        fm_to_agv_comm = dict()
        [fm_to_agv_comm.update({i + 1: simpy.FilterStore(env)}) for i in range(num_robots)]

        # Define communication channel between AGVs and fleet manager (These are just the virtual IP adresses)
        agv_to_fm_comm = simpy.FilterStore(env)
        
        # Define MES
        mes = MES(env, kb, order_list, print_)

        # Define Fleet Manger / Central Auctioneer
        FleetManager(env, kb, fm_to_agv_comm, agv_to_fm_comm, print_)

        # Define AGVs
        for ID in range(num_robots):
            agv_params = {'ID': ID + 1,
                          'robot_speed': self.robot_speed,
                          'task_execution_time': self.task_execution_time,
                          'start_location': self.start_locations[ID % 3],
                          'depot_locations': self.depot_locations,
                          'battery_threshold': self.battery_threshold,
                          'collision_threshold': self.collision_threshold,
                          'max_charging_time': self.max_charging_time,
                          'max_tasks_in_task_list': self.max_tasks_in_task_list,
                          'epsilon': self.epsilon,
                          'initial_resources': self.initial_resources}
            AGV(env, agv_params, kb, fm_to_agv_comm[ID + 1], agv_to_fm_comm, print_)
        
        # Define logger
        if log_:
            Logger(env, kb, print_)
        
        # Define online renderer
        if render_:
            RendererOnline(env, kb, self.depot_locations, self.charge_locations, print_)
        
        # Run environment untill all tasks are executed
        env.run(until=mes.main)
        simulation_time = env.now - 1  # Correction for last time step

        # Print simulation duration
        with open("../logfiles/simulation_information.txt", "w") as file:
            file.write(str(num_robots) + '\n')
            file.write(str(simulation_time) + '\n')

        # Get simulation summary
        travel_cost = sum(get_travel_cost_per_robot())
        charging_cost = sum(get_charging_cost_per_robot())
        congestions = sum(get_congestions_per_robot()) / 2

        return simulation_time, travel_cost, charging_cost, congestions

    def generate_situation(self, number_of_tasks, orders_file):
        generate_situation(number_of_tasks, self.setup_file_path, orders_file)
    
    # Creates the central knowledge base
    def define_knowledge_base(self, env):
        global_task_list = simpy.FilterStore(env)  # This needs to be a table in SQL database
        global_robot_list = simpy.FilterStore(env)  # This needs to be a table in SQL database
        tasks_executing = simpy.FilterStore(env)  # This needs to be a table in SQL database
        kb = dict()
        kb['global_task_list'] = global_task_list
        kb['global_robot_list'] = global_robot_list
        kb['tasks_executing'] = tasks_executing
        kb['charge_locations'] = self.charge_locations
        kb['graph'] = self.graph
        return kb
