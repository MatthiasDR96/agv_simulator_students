import simpy

from src.agv.AGV_Action import Action
from src.agv.AGV_Comm import Comm
from src.agv.AGV_ResourceManagement import ResourceManagement
from src.agv.AGV_TaskAllocation import TaskAllocation
from src.datatypes.Robot import Robot
from src.utils.utils import *


class AGV:
    """
        A class containing the intelligence of the AGV agent
    """

    def __init__(self, env, agv_params, kb, fm_to_agv_comm, agv_to_fm_comm, print_):
    
        # Simulation environment
        self.env = env

        # Printing
        self.print = print_
    
        # Communication attributes
        self.ip = '172.21.0.0'
        self.kb = kb  # Knwoledge base (SQL database)
        self.fm_to_agv_comm = fm_to_agv_comm
        self.agv_to_fm_comm = agv_to_fm_comm
        self.comm = Comm(self.ip)

        # agv attributes
        self.ID = agv_params['ID']  # Each agv has an unique ID number
        self.robot_speed = agv_params['robot_speed']  # Constant robot speed
        self.task_execution_time = agv_params['task_execution_time']
        self.depot_locations = agv_params['depot_locations']
        self.battery_threshold = agv_params['battery_threshold']  # Battery threshold when the agv needs to charge
        self.collision_threshold = agv_params['collision_threshold']  # Collision threshold
        self.max_charging_time = agv_params['max_charging_time']  # One hour to charge fully
        self.epsilon = agv_params['epsilon']  # Objective parameter
        self.max_tasks_in_task_list = agv_params['max_tasks_in_task_list']
        self.initial_resources = agv_params['initial_resources']

        # Local task list
        self.kb['local_task_list_R' + str(self.ID)] = simpy.FilterStore(self.env)

        # Updated attributes
        self.robot_location = self.kb['graph'].nodes[agv_params['start_location']].pos
        self.robot_node = agv_params['start_location']
        self.status = 'IDLE'
        self.battery_status = self.initial_resources
        self.travelled_time = 0
        self.charged_time = 0
        self.heading_direction = 0
        self.task_executing = None
        self.path = []
        self.slots = []
        self.congestions = 0
        self.total_path = []
    
        # AGV tasks
        self.task_allocation = TaskAllocation(self)
        self.resource_management = ResourceManagement(self)
        self.action = Action(self)
        
        # Processes
        self.main = self.env.process(self.main())

        # Initialize
        self.my_print("agv " + str(self.ID) + ":                   Started")
        self.robot = Robot(self.ID, self.robot_location, self.robot_node, self.heading_direction, self.path,
                           self.status, self.battery_status, self.travelled_time, self.charged_time, self.congestions,
                           self.task_executing)
        self.comm.sql_write(self.kb['global_robot_list'], self.robot)
    
    def main(self):
    
        while True:
        
            # Wait for an assigned task
            self.my_print("AGV " + str(self.ID) + ":      Waiting for tasks..." + " at " + str(self.env.now))
            self.task_executing = yield self.kb['local_task_list_R' + str(self.ID)].get()
        
            # Start task
            self.my_print("AGV " + str(self.ID) + ":      Start executing task " + str(self.task_executing.to_string())
                          + " at " + str(self.env.now))
            if self.status is not 'EMPTY':
                self.status = 'BUSY'
            self.update_global_robot_list()

            # Remove task from global task list and add to executing list
            self.comm.sql_remove_task(self.kb['global_task_list'], self.task_executing.order_number)
            self.task_executing.robot = self.ID
            self.comm.sql_write(self.kb['tasks_executing'], self.task_executing)
                
            # Go to task A
            yield self.env.process(self.execute_task(self.task_executing))

            # Perform task A
            yield self.env.process(self.action.pick())
            self.task_executing.picked = True
            self.my_print("AGV " + str(self.ID) + ":      Picked item of task " + str(self.task_executing.order_number) + " at "
                  + str(self.env.now))

            # Task executed
            self.comm.sql_remove_task(self.kb['tasks_executing'], self.task_executing.order_number)
            self.task_executing = None

            # Set status to IDLE when task is done or when done charging
            if self.status is not 'EMPTY':
                self.status = 'IDLE'
            self.update_global_robot_list()

    # Calculates shortest path with Astar and executes
    def execute_task(self, task):
    
        # Compute astar path
        self.path, _ = find_shortest_path(self.kb['graph'], self.robot_node, task.pos_A)

        # Update state
        self.update_global_robot_list()

        # Move from node to node
        while len(self.path) > 0:
            yield self.env.process(self.action.move_to_node(self.path[0]))

        # Check if task is charging task
        if task.order_number == '000':

            # Compute charging time
            charging_time = (100 - self.battery_status) / self.resource_management.charging_factor

            # Update status
            self.my_print(
                "agv " + str(self.ID) + ":      Is charging for " + str(charging_time) + " seconds at " + str(
                    self.env.now))
            self.status = 'CHARGING'
            self.update_global_robot_list()

            # Charging
            yield self.env.timeout(charging_time)

            # Update robot status
            self.battery_status = self.battery_status + charging_time * self.resource_management.charging_factor
            self.status = 'IDLE'
            self.charged_time += int(charging_time) + calculate_path_traveltime(self.kb['graph'], self.path,
                                                                                    self.robot_speed)
            self.update_global_robot_list()

    def update_global_robot_list(self):
        self.comm.sql_remove_robot(self.kb['global_robot_list'], self.ID)
        self.robot = Robot(self.ID, self.robot_location, self.robot_node, self.heading_direction, self.path,
                           self.status, self.battery_status, self.travelled_time, self.charged_time, self.congestions,
                           self.task_executing, self.total_path)
        self.comm.sql_write(self.kb['global_robot_list'], self.robot)

    def my_print(self, msg):
        if self.print:
            print(msg)