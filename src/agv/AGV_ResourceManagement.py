from src.datatypes.Task import Task
from src.solvers.astar_solver import *


class ResourceManagement:
    """
            A class containing the intelligence of the Resource Management agent
    """
    
    def __init__(self, agv):
        
        # AGV
        self.agv = agv

        # Params
        self.resource_scale_factor = 0.3
        self.charging_factor = 1  # % per second

        # Process
        self.status_manager = self.agv.env.process(self.status_manager())

    def status_manager(self):
        while True:
            yield self.agv.env.timeout(0.1)  # Sample time
            if self.agv.status == 'BUSY':
                if self.agv.battery_status < self.agv.battery_threshold:

                    # Update status to empty
                    self.agv.status = 'EMPTY'
                    self.agv.update_global_robot_list()

                    # Get start node of robot
                    start_node = self.agv.task_executing.pos_A if self.agv.task_executing else self.agv.robot_node

                    # Create charging task
                    closest_charging_station = self.search_closest_charging_station(start_node)
                    charging_task = Task('000', closest_charging_station, 0)

                    # Get tasks in new local task list
                    new_local_task_list = [charging_task] + [task for task in
                                                             self.agv.kb['local_task_list_R' + str(self.agv.ID)].items]

                    # Delete local task list
                    for i in range(len(self.agv.kb['local_task_list_R' + str(self.agv.ID)].items)):
                        self.agv.kb['local_task_list_R' + str(self.agv.ID)].get()

                    # Put optimal task_sequence in local task list
                    for task in new_local_task_list:
                        self.agv.comm.sql_write(self.agv.kb['local_task_list_R' + str(self.agv.ID)], task)

    def search_closest_charging_station(self, location):
        closest_point = None
        min_distance = float('inf')
        for name in self.agv.depot_locations:
            _, distance = find_shortest_path(self.agv.kb['graph'], location, name)
            if distance < min_distance:
                min_distance = distance
                closest_point = name
        return closest_point

    def resource_consumption(self, travel_time):
        charge_loss = self.resource_scale_factor * travel_time
        return charge_loss
