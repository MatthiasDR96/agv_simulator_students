from Bid import Bid
from TravelingSalesman import TravelingSalesman


class TaskAllocation:
    """
            A class containing the intelligence of the Task Allocation agent
    """
    
    def __init__(self, agv):
        
        # AGV
        self.agv = agv
        
        # Traveling salesman
        self.tsp = TravelingSalesman(agv.astar)
        
        # Process
        self.task_collector = self.agv.env.process(self.task_collector())
    
    def task_collector(self):
    
        while True:
            
            # Wait for auctioneer to announce or assign a task
            task = yield self.agv.fm_to_agv_comm.get()
            
            if task.message == 'announce':
                
                # Compute bid for the task
                bid = self.compute_bid_marginal(task)
                
                # Send bid to the auctioneer
                self.agv.comm.tcp_write(self.agv.agv_to_fm_comm, bid)
                # print("AGV " + str(self.ID) + ":      Sent bid " + str(bid.value) + " to auctioneer at "
                # + str(self.env.now))
            
            else:
                # Add assigned task to local task list
                print("AGV " + str(self.agv.ID) + ":      Added task " + task.to_string() + " to local task list at "
                      + str(self.agv.env.now))
                self.update_local_task_list_tsp(task)
    
    def compute_bid_simple(self, task):
        
        # Compute cost to execute task
        bid = Bid()
        distance_ra, _ = self.agv.astar.find_shortest_path(self.agv.robot_location, task.pos_A)
        distance_ab, _ = self.agv.astar.find_shortest_path(task.pos_A, task.pos_B)
        bid.value = distance_ra + distance_ab
        bid.robot = self.agv.robot
        bid.task = task
        return bid
    
    def compute_bid_marginal(self, task):
        
        # Get tasks in local task list
        print("Compute bid:     new task: " + str(task.to_string()))
        local_tasks = [task for task in self.agv.local_task_list.items]

        # Compute cost of current tour
        cost_of_current_tour = self.compute_cost_of_current_tour(local_tasks, self.agv.robot_location)
        print("Compute bid:     cost old tour: " + 'distance: ' + str(cost_of_current_tour) + " tour: "
              + str([task.to_string() for task in local_tasks]))
        
        # Append new task in local task list
        extra_tasks = local_tasks + [task]
        print("Compute bid:     local task list extra: " + str([task.to_string() for task in extra_tasks]))

        # Compute traveling salesman to obtain optimized task sequence with new task
        optimal_tour, travel_time = self.tsp.compute_tsp(extra_tasks, self.agv.robot_location)
        print("Compute bid:     tsp tour: " + str([task.to_string() for task in optimal_tour]))
        
        # Insert charging station if needed
        optimal_tour, optimal_charging_time, fitness = \
            self.agv.resource_management.compute_optimal_insertion_and_charging_time(optimal_tour, self.agv.battery_status)
        
        # Compute cost of new tour
        cost_of_new_tour = self.compute_cost_of_current_tour(optimal_tour, self.agv.robot_location)
        cost_of_new_tour += optimal_charging_time
        print("Compute bid:     cost new tour: " + 'distance: ' + str(cost_of_new_tour) + " tour: "
              + str([task.to_string() for task in optimal_task_sequence]))
        
        # Marginal cost to execute task
        marginal_cost = cost_of_new_tour - cost_of_current_tour
        print("Compute bid:     marginal cost: " + str(marginal_cost))
        
        # Make bid
        bid = Bid()
        bid.value = marginal_cost
        bid.robot = self.agv.robot
        bid.task = task
        
        return bid
    
    def compute_cost_of_current_tour(self, tour, robot_location):
        cost = 0
        # If the AGV is executing a task, insert that cost plus cost of going from that task to first in local list
        if self.agv.task_executing:
            # If task is not picked yet, consider cost of going from robot location to pick-up location
            if not self.agv.task_executing.picked:
                cost_r1, _ = self.agv.astar.find_shortest_path(robot_location, self.agv.task_executing.pos_A)
                cost_12, _ = self.agv.astar.find_shortest_path(self.agv.task_executing.pos_A,
                                                               self.agv.task_executing.pos_B)
                cost += cost_r1 + cost_12
            # If task is picked, consider cost of going from robot location to drop-off location
            else:
                cost_r2, _ = self.agv.astar.find_shortest_path(robot_location, self.agv.task_executing.pos_B)
                cost += cost_r2
            # If tour is not empty, consider cost of going from executing drop-off location to first location of tour
            if tour:
                cost_23, _ = self.agv.astar.find_shortest_path(self.agv.task_executing.pos_B, tour[0].pos_A)
                cost += cost_23
        # If the AGV is not executing a task, consider cost of going from robot location to first location in tour
        else:
            if tour:
                cost_r3, _ = self.agv.astar.find_shortest_path(robot_location, tour[0].pos_A)
                cost += cost_r3
        # If tour is not empty, consider cost of whole tour
        if tour:
            for i in range(len(tour)-1):
                distance_r1, _ = self.agv.astar.find_shortest_path(tour[i].pos_A, tour[i].pos_B)
                distance_12, _ = self.agv.astar.find_shortest_path(tour[i].pos_B, tour[i+1].pos_A)
                cost += distance_r1 + distance_12
            # Consider cost of going from last task pick-up to last task drop-off
            distance_12, _ = self.agv.astar.find_shortest_path(tour[-1].pos_A, tour[-1].pos_B)
            cost += distance_12
        return cost
        
    def update_local_task_list_simple(self, task):
        
        # Put task in queue
        self.agv.local_task_list.put(task)
    
    def update_local_task_list_tsp(self, task):
        
        # Get tasks in local task list
        local_tasks = [task for task in self.agv.local_task_list.items]
        
        # Append new task
        extra_tasks = local_tasks + [task]
        
        # Compute traveling salesman
        optimal_task_sequence = self.tsp.compute_tsp(extra_tasks, self.agv.robot_location)
        
        # Insert charging station if needed
        # optimal_task_sequence_charge, charging_time = \
            # self.agv.resource_management.compute_optimal_insertion(optimal_task_sequence, self.agv.battery_status)
        
        # Delete local task list
        for i in range(len(self.agv.local_task_list.items)):
            self.agv.local_task_list.get()

        # Put optimal tour in local task list
        for task_ in optimal_task_sequence:
            self.agv.local_task_list.put(task_)
