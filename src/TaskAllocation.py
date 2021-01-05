class TaskAllocation:
    """
            A class containing the intelligence of the Task Allocation agent
    """
    
    def __init__(self, agv):
        
        # AGV
        self.agv = agv
        
        # Process
        self.task_collector = self.agv.env.process(self.task_collector())
    
    # Executes tasks when assigned from FleetManager
    def task_collector(self):
    
        while True:
            
            # Wait for fleetmanager to announce or assign a task
            task = yield self.agv.fm_to_agv_comm.get()
            
            # Add assigned task to local task list
            print("AGV " + str(self.agv.ID) + ":      Added task " + task.to_string() + " to local task list at "
                      + str(self.agv.env.now))
            self.update_local_task_list(task)
        
    # Adds assigned task to local task list
    def update_local_task_list(self, task):
        
        # Put task in queue
        self.agv.local_task_list.put(task)