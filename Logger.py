import logging
import logging.handlers

import numpy as np


class Logger:

    def __init__(self, env, global_task_list, global_robot_list, local_task_lists, tasks_executing):

        # Attributes
        self.env = env
        self.global_robot_list = global_robot_list
        self.global_task_list = global_task_list
        self.local_task_lists = local_task_lists
        self.tasks_executing = tasks_executing

        # Process
        self.logging = self.env.process(self.logging())

    def logging(self):

        # Create loggers
        log_filename_1 = 'logfiles/global_task_list.txt'
        log_filename_2 = 'logfiles/local_task_lists.txt'
        log_filename_3 = 'logfiles/tasks_executing.txt'
        log_filename_4 = 'logfiles/global_robot_list.txt'

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
            for task in self.global_task_list.items:
                global_task_list_string += task.to_log() + ":"

            # Local task lists
            local_task_lists_string = ""
            for i in range(len(self.local_task_lists)):
                list = str(i + 1) + ',' + str([task.to_log() for task in self.local_task_lists[i].items])
                local_task_lists_string += list + ":"

            # Task executing
            tasks_executing_string = ""
            for task in self.tasks_executing.items:
                tasks_executing_string += task.to_log() + ":"

            # Global robot list
            global_robot_list_string = ""
            robots = np.copy(self.global_robot_list.items)
            robots = sorted(robots, key=lambda robot_: robot_.ID)
            for robot in robots:
                global_robot_list_string += robot.to_log() + ":"

            # Logging
            log1.debug(str(self.env.now) + "/" + str(global_task_list_string))
            log2.debug(str(self.env.now) + "/" + str(local_task_lists_string))
            log3.debug(str(self.env.now) + "/" + str(tasks_executing_string))
            log4.debug(str(self.env.now) + "/" + str(global_robot_list_string))