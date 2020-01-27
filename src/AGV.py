import simpy

from Action import Action
from Astar import Astar
from Comm import Comm
from MotionPlanning import MotionPlanning
from ResourceManagement import ResourceManagement
from Robot import Robot
from TaskAllocation import TaskAllocation


class AGV:
    """
        A class containing the intelligence of the AGV agent
    """

    def __init__(self, env, agv_params, kb, fm_to_agv_comm, agv_to_fm_comm):
    
        # Simulation environment
        self.env = env
    
        # Communication attributes
        self.kb = kb
        self.fm_to_agv_comm = fm_to_agv_comm
        self.agv_to_fm_comm = agv_to_fm_comm
    
        # AGV attributes
        self.ID = agv_params['ID']  # Each AGV has an unique ID number
        self.robot_speed = agv_params['robot_speed']  # Constant robot speed
        self.task_execution_time = agv_params['task_execution_time']
        self.robot_location = agv_params['start_location']  # Current AGV location
        self.battery_threshold = agv_params['battery_threshold']  # Battery threshold when the AGV needs to charge
        self.max_charging_time = agv_params['max_charging_time']  # One hour to charge fully
    
        # Local task list
        self.local_task_list = simpy.FilterStore(self.env)
        self.kb['local_task_list_R' + str(self.ID)] = simpy.FilterStore(self.env)
        
        # Layout attributes
        self.charging_stations = kb['charge_locations']
        self.astar = Astar(self.kb['graph'])  # Astar shortest path planner
    
        # Updated attributes
        self.heading_direction = 0  # Direction towards which the AGV is driving
        self.path = []  # Current path to execute
        self.cost_of_current_tour = 0
    
        # Monitoring attributes
        self.status = 'IDLE'  # Current AGV status
        self.battery_status = 100  # Current battery status of the AGV
    
        # AGV tasks
        self.task_allocation = TaskAllocation(self)
        self.motion_planning = MotionPlanning(self)
        self.resource_management = ResourceManagement(self)
        self.action = Action(self)
        self.comm = Comm()
        
        # Processes
        self.main = self.env.process(self.main())
        self.status_updater = self.env.process(self.status_updater())
        self.status_manager = self.env.process(self.status_manager())
    
        # Initialize
        print("AGV " + str(self.ID) + ":                   Started")
        self.robot = Robot(self.ID, self.robot_location)
        self.comm.sql_write(self.kb['global_robot_list'], self.robot)
    
    def main(self):
    
        while True:
        
            # Wait for an assigned task
            print("AGV " + str(self.ID) + ":      Waiting for tasks..." + " at " + str(self.env.now))
            task = yield self.local_task_list.get()
        
            if task:
                # Start task
                print("AGV " + str(self.ID) + ":      Start executing task " + str(task.to_string()) +
                      " at " + str(self.env.now))
                self.status = 'BUSY'

                # Remove task from global task list and add to executing list
                self.comm.sql_remove(self.kb['global_task_list'], task.order_number)
                task.robot = self.ID
                self.comm.sql_write(self.kb['tasks_executing'], task)
                
                # Go to task A
                yield self.env.process(self.execute_path(task.pos_A))

                # Perform task A
                yield self.env.process(self.action.pick())
                task.picked = True
                print("AGV " + str(self.ID) + ":      Picked item of task " + str(task.order_number) + " at "
                      + str(self.env.now))

                # Go to task B
                yield self.env.process(self.execute_path(task.pos_B))

                # Perform task B
                yield self.env.process(self.action.place())
                print("AGV " + str(self.ID) + ":      Dropped item of task " + str(task.order_number) + " at " +
                      str(self.env.now))

                # Task executed
                self.comm.sql_remove(self.kb['tasks_executing'], task.order_number)
            
            # If battery threshold exceeded, go to closest charging station and charge fully
            if self.status == 'EMPTY':
                yield self.env.process(self.resource_management.charge_classic())
            
            # Set status to IDLE when task is done or when done charging
            self.status = 'IDLE'

    def execute_path(self, goal_position):
    
        # Compute astar path
        start_position = (int(self.robot_location[0]), int(self.robot_location[1]))
        distance, path = self.astar.find_shortest_path(start_position, goal_position)
        self.path = path[1:]
        # print("Path to follow: " + str([node.pos for node in self.path]))
    
        # Move from node to node
        while len(self.path) > 0:
            node = self.path[0]
            yield self.env.process(self.action.move_to_node(node))
    
    def status_manager(self):
        while True:
            yield self.env.timeout(1)  # Sample time
            if not self.status == 'CHARGING':
                if self.battery_status < self.battery_threshold:
                    self.status = 'EMPTY'

    def status_updater(self):
        while True:
    
            # Update global robot list
            self.kb['global_robot_list'].get(lambda robot_: robot_.ID == self.ID)  # Should be with comm.sql_remove
            self.robot = Robot(self.ID, self.robot_location, self.heading_direction, self.path, self.status,
                               self.battery_status)
            self.comm.sql_write(self.kb['global_robot_list'], self.robot)
            
            # Update local task lists
            for i in self.kb['local_task_list_R' + str(self.ID)].items:
                self.kb['local_task_list_R' + str(self.ID)].get()
            self.kb['local_task_list_R' + str(self.ID)].get()
            [self.comm.sql_write(self.kb['local_task_list_R' + str(self.ID)], task)
             for task in self.local_task_list.items]
    
            # Sample time
            yield self.env.timeout(1)
