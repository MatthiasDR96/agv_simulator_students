import logging
import logging.handlers

import numpy as np

import Comm as comm


class Logger:
    """
            A class containing the logger methods which log relevant data to logger files
    """
    
    def __init__(self, env, kb):

        # Attributes
        self.env = env
        self.kb = kb
    
        # Process
        self.logging = self.env.process(self.logging())

    def logging(self):
    
        # Create logger files
        log_filename_1 = '../logfiles/global_task_list.txt'
        log_filename_2 = '../logfiles/local_task_list.txt'
        log_filename_3 = '../logfiles/tasks_executing.txt'
        log_filename_4 = '../logfiles/global_robot_list.txt'
    
        # Create logger 1
        log1 = logging.getLogger('MyLogger1')
        log1.setLevel(logging.DEBUG)
        handler1 = logging.handlers.RotatingFileHandler(
            log_filename_1, backupCount=1, mode='w')
        log1.addHandler(handler1)
    
        # Create logger 2
        log2 = logging.getLogger('MyLogger2')
        log2.setLevel(logging.DEBUG)
        handler2 = logging.handlers.RotatingFileHandler(
            log_filename_2, backupCount=1, mode='w')
        log2.addHandler(handler2)
    
        # Create logger 3
        log3 = logging.getLogger('MyLogger3')
        log3.setLevel(logging.DEBUG)
        handler3 = logging.handlers.RotatingFileHandler(
            log_filename_3, backupCount=1, mode='w')
        log3.addHandler(handler3)
    
        # Create logger 4
        log4 = logging.getLogger('MyLogger4')
        log4.setLevel(logging.DEBUG)
        handler4 = logging.handlers.RotatingFileHandler(
            log_filename_4, backupCount=1, mode='w')
        log4.addHandler(handler4)
    
        # Logging loop
        while True:
    
            yield self.env.timeout(1)  # Sample time

            # Global tasks list
            global_task_list_string = ""
            global_task_list = comm.sql_read(self.kb['global_task_list'])
            for task in global_task_list:
                global_task_list_string += task.to_log() + ":"

            # Local task lists
            local_task_list_string = ""
            for key in self.kb.keys():
                if "local_task_list_R" in key:
                    local_task_list_string += str(key[-1]) + ";"
                    local_task_list = comm.sql_read(self.kb[key])
                    for task in local_task_list:
                        local_task_list_string += str(task.to_log()) + ":"

            # Task executing
            tasks_executing_string = ""
            tasks_executing = comm.sql_read(self.kb['tasks_executing'])
            for task in tasks_executing:
                tasks_executing_string += task.to_log() + ":"

            # Global robot list
            global_robot_list_string = ""
            global_robot_list = comm.sql_read(self.kb['global_robot_list'])
            robots = np.copy(global_robot_list)
            robots = sorted(robots, key=lambda robot_: robot_.ID)
            for robot in robots:
                global_robot_list_string += robot.to_log() + ":"
    
            # Logging to logging files
            log1.debug(str(self.env.now) + "/" + str(global_task_list_string))
            log2.debug(str(self.env.now) + "/" + str(local_task_list_string))
            log3.debug(str(self.env.now) + "/" + str(tasks_executing_string))
            log4.debug(str(self.env.now) + "/" + str(global_robot_list_string))
