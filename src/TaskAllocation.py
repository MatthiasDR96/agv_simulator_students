from Bid import Bid
from Comm import Comm
from TravelingSalesman import TravelingSalesman


class TaskAllocation:
    """
            A class containing the intelligence of the Task Allocation agent
    """
    
    def __init__(self, agv):
        
        # AGV
        self.agv = agv
        self.comm = Comm()
        
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
                bid = self.compute_bid_simple(task)
                
                # Send bid to the auctioneer
                self.comm.tcp_write(self.agv.agv_to_fm_comm, bid)
                # print("AGV " + str(self.ID) + ":      Sent bid " + str(bid.value) + " to auctioneer at "
                # + str(self.env.now))
            
            else:
                # Add assigned task to local task list
                print("AGV " + str(self.agv.ID) + ":      Added task " + task.to_string() + " to local task list at "
                      + str(self.agv.env.now))
                self.update_local_task_list_simple(task)
    
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
        tasks = [task for task in self.agv.local_task_list.items]
        print("Compute bid local task list: " + str([task.to_string() for task in tasks]))
        
        # Append new task
        tasks.append(task)
        print("Compute bid local task list extra: " + str([task.to_string() for task in tasks]))
        
        # Compute traveling salesman
        distance, optimal_task_sequence = self.tsp.compute_tsp(tasks, self.agv.robot_location)
        print("Compute bid result: " + 'distance: ' + str(distance) + " tour: "
              + str([task.to_string() for task in optimal_task_sequence]))
        
        # Marginal cost to execute task
        cost_of_current_tour = 0  # TODO calculate current cost to execute local task list
        marginal_cost = distance - cost_of_current_tour
        print("Compute bid marginal cost: " + str(marginal_cost))
        
        # Make bid
        bid = Bid()
        bid.value = marginal_cost
        bid.robot = self.agv.robot
        bid.task = task
        
        return bid
    
    def update_local_task_list_simple(self, task):
        
        # Put task in queue
        self.agv.local_task_list.put(task)
    
    def update_local_task_list_tsp(self, task):
        
        # Get tasks in local task list
        tasks = [task for task in self.agv.local_task_list.items]
        
        # Append new task
        tasks.append(task)
        
        # Compute traveling salesman
        distance, optimal_task_sequence = self.tsp.compute_tsp(tasks, self.agv.robot_location)
        cost_of_current_tour = distance
        
        # Delete local task list
        for i in self.agv.local_task_list.items:
            self.agv.local_task_list.get()
        
        # Put optimal tour in local task list
        for task_ in optimal_task_sequence:
            self.agv.local_task_list.put(task_)
        
        # TODO Check ordering local_task_list
