import numpy as np

import Comm as comm


class CentralAuctioneer:
    
    def __init__(self, env, kb, fm_to_agv_comm, agv_to_fm_comm):
        
        # Attributes
        self.env = env
        self.kb = kb
        self.fm_to_agv_comm = fm_to_agv_comm
        self.agv_to_fm_comm = agv_to_fm_comm
        self.fitness_file = open("../logfiles/fitness_values.txt", "w")
        
        # Process
        self.main = self.env.process(self.main())
        
        # Initialize
        print("Central auctioneer:      Started")
    
    def main(self):
        
        print("\n")
        while True:
            
            # Wait for task on global task list
            task = yield self.kb['global_task_list'].get()
            task.message = "announce"
            
            # Get available
            available_robots = self.define_current_status()
            
            # Announce task to robots
            for robot in available_robots:
                comm.tcp_write(self.fm_to_agv_comm[robot.ID], task)
                # print("Central auctioneer:      Announced task " + task.to_string() + " to robot " + str(robot.ID) +
                # " at " + str(self.env.now))
            
            # Wait for bids
            bids = []
            while len(bids) < len(available_robots):
                bid = yield self.agv_to_fm_comm.get()
                # print("Central auctioneer:      Received bid with value " + str(bid.value) + " from robot " +
                # str(bid.robot.ID) + " at " + str(self.env.now))
                bids.append(bid)
            
            # Compute best bid
            bids.sort(key=lambda p: p.value)
            best_bid = bids[0]
            print("Central auctioneer:      Best_bid " + str(best_bid.value) + ' from robot ' + str(best_bid.robot.ID))
            
            # Send task to robots
            robot = best_bid.robot
            best_bid.task.message = 'assign'
            best_bid.task.robot = robot.ID
            comm.tcp_write(self.fm_to_agv_comm[robot.ID], best_bid.task)
            # print("Central auctioneer:      Sent task " + task.to_string() + " to robot " + str(robot.ID) + " at "
            # + str(self.env.now))
    
    def define_current_status(self):
        
        # Copy robot list
        robots = np.copy(comm.sql_read(self.kb['global_robot_list']))
        robots = sorted(robots, key=lambda robot_: robot_.ID)
        
        return robots
