import logging
import logging.handlers

import numpy as np

from src.agv.AGV_Comm import Comm


class Logger:

    def __init__(self, env, kb, print_):

        # Attributes
        self.env = env

        # Printing
        self.print = print_

        # Communication attributes
        self.ip = '172.21.0.0'
        self.kb = kb
        self.comm = Comm(self.ip)

        # Process
        self.logging = self.env.process(self.logging())

        # Initialize
        self.my_print("Logger:                  Started")

    def logging(self):

        # Create loggers
        log_filename_1 = '../logfiles/global_task_list.txt'
        log_filename_2 = '../logfiles/local_task_list.txt'
        log_filename_3 = '../logfiles/tasks_executing.txt'
        log_filename_4 = '../logfiles/global_robot_list.txt'

        log1 = logging.getLogger('MyLogger1')
        log1.setLevel(logging.DEBUG)
        handler1 = logging.handlers.RotatingFileHandler(
            log_filename_1, backupCount=1, mode='w')
        log1.addHandler(handler1)

        log2 = logging.getLogger('MyLogger2')
        log2.setLevel(logging.DEBUG)
        handler2 = logging.handlers.RotatingFileHandler(
            log_filename_2, backupCount=1, mode='w')
        log2.addHandler(handler2)

        log3 = logging.getLogger('MyLogger3')
        log3.setLevel(logging.DEBUG)
        handler3 = logging.handlers.RotatingFileHandler(
            log_filename_3, backupCount=1, mode='w')
        log3.addHandler(handler3)

        log4 = logging.getLogger('MyLogger4')
        log4.setLevel(logging.DEBUG)
        handler4 = logging.handlers.RotatingFileHandler(
            log_filename_4, backupCount=1, mode='w')
        log4.addHandler(handler4)

        while True:
            yield self.env.timeout(1)

            # Global tasks list
            global_task_list_string = ""
            global_task_list = self.comm.sql_read(self.kb['global_task_list'])
            for task in global_task_list:
                global_task_list_string += task.to_log() + "|"

            # Local task lists
            local_task_list_string = ""
            for key in self.kb.keys():
                if "local_task_list_R" in key:
                    local_task_list_string += "["
                    local_task_list = self.comm.sql_read(self.kb[key])
                    for task in local_task_list:
                        local_task_list_string += str(task.to_log()) + ':'
                    local_task_list_string = local_task_list_string
                    local_task_list_string += "]|"

            # Task executing
            tasks_executing_string = ""
            tasks_executing = self.comm.sql_read(self.kb['tasks_executing'])
            for task in tasks_executing:
                tasks_executing_string += task.to_log() + "|"

            # Global robot list
            global_robot_list_string = ""
            global_robot_list = self.comm.sql_read(self.kb['global_robot_list'])
            robots = np.copy(global_robot_list)
            robots = sorted(robots, key=lambda robot_: robot_.ID)
            for robot in robots:
                global_robot_list_string += robot.to_log() + "|"

            # Logging
            log1.debug(str(self.env.now) + "|" + str(global_task_list_string))
            log2.debug(str(self.env.now) + "|" + str(local_task_list_string))
            log3.debug(str(self.env.now) + "|" + str(tasks_executing_string))
            log4.debug(str(self.env.now) + "|" + str(global_robot_list_string))

    def my_print(self, msg):
        if self.print:
            print(msg)
