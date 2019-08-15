import logging
import logging.handlers
import math

import matplotlib
import matplotlib.pyplot as plt
import numpy as np


class Renderer:

    def __init__(self, env, locations, global_task_list, global_robot_list, local_task_lists, tasks_executing,
                 depot_locations, charge_locations):

        # Attributes
        self.env = env
        self.locations = locations
        self.global_robot_list = global_robot_list
        self.global_task_list = global_task_list
        self.local_task_lists = local_task_lists
        self.status_monitor = [[] for robot in global_robot_list.items]
        self.battery_status_monitor = [[] for robot in global_robot_list.items]
        self.tasks_executing = tasks_executing
        self.depot_locations = depot_locations
        self.charge_locations = charge_locations
        self.colors = matplotlib.cm.rainbow(np.linspace(0, 1, len(global_robot_list.items)))

        # Processes
        self.render_scene = self.env.process(self.render_scene())
        self.status_monitor_update = self.env.process(self.status_monitor_update())
        self.battery_status_monitor_update = self.env.process(self.battery_status_monitor_update())
        self.logging = self.env.process(self.logging())

        # Initiate plt
        plt.close("all")
        plt.rcParams['toolbar'] = 'None'
        plt.figure("AGV Simulator", figsize=(11, 6), facecolor='k', edgecolor='k', dpi=100)
        plt.title("AGV Simulator")

    def render_scene(self):

        while True:

            yield self.env.timeout(1)
            number_agvs = len(self.global_robot_list.items)
            plt.subplot2grid((1, number_agvs + 3), (0, 0), colspan=3)
            plt.subplots_adjust(wspace=1.5, hspace=0.4)
            plt.title("AGV Layout")
            plt.style.use('dark_background')

            # Plot Graph
            for node in self.locations:
                if node.pos in self.depot_locations:
                    plt.plot(node.pos[0], node.pos[1], 'g^', ms=6)
                elif node.pos in self.charge_locations:
                    plt.plot(node.pos[0], node.pos[1], 'g^', ms=6)
                else:
                    plt.plot(node.pos[0], node.pos[1], 'b.', ms=6)
                    # plt.text(node.pos[0]-1, node.pos[1]+1, node.name)
                    for neighbor in node.neighbors:
                        plt.plot([node.pos[0], neighbor.pos[0]], [node.pos[1], neighbor.pos[1]], 'b-', lw=0.5)

            # Plot AGVs
            for robot in self.global_robot_list.items:
                color = self.colors[robot.ID - 1]
                plt.plot(robot.position[0], robot.position[1], color=color, marker='o', ms=10)
                plt.arrow(robot.position[0], robot.position[1], math.cos(robot.heading_direction),
                          math.sin(robot.heading_direction), width=0.3, color=color)
                plt.text(robot.position[0] + 0.5, robot.position[1] + 1.5, str("AGV" + str(robot.ID)))
                if robot.path:
                    plt.plot([robot.position[0], robot.path[0].pos[0]], [robot.position[1], robot.path[0].pos[1]],
                             color=color, lw=1.0)
                    for i in range(len(robot.path)):
                        plt.plot(robot.path[i].pos[0], robot.path[i].pos[1], color=color, marker='.', ms=10)
                        if i < len(robot.path) - 1:
                            plt.plot([robot.path[i].pos[0], robot.path[i + 1].pos[0]],
                                     [robot.path[i].pos[1], robot.path[i + 1].pos[1]], color=color, lw=1.5)

            # Plot tasks not executing
            for task in self.global_task_list.items:
                plt.plot(task.pos_A[0], task.pos_A[1], 'gs', ms=7)
                plt.text(task.pos_A[0] - 1, task.pos_A[1] + 1.5, str(task.order_number))

            # Plot tasks executing
            for task in self.tasks_executing.items:
                if not task.picked:
                    plt.plot(task.pos_A[0], task.pos_A[1], 'gs', ms=7)
                    plt.text(task.pos_A[0] - 1, task.pos_A[1] + 1.5, str(task.order_number))
                else:
                    for robot in self.global_robot_list.items:
                        if task.robot == robot.ID:
                            plt.plot(robot.position[0], robot.position[1], 'gs', ms=7)
                            plt.text(robot.position[0] - 1, robot.position[1] + 1.5, str(task.order_number))

            # Plot
            plt.draw()
            plt.pause(0.0001)
            plt.clf()
            plt.tight_layout()

    def status_monitor_update(self):
        while True:
            yield self.env.timeout(1)
            robots = self.global_robot_list.items
            robots = sorted(robots, key=lambda robot: robot.ID)
            for i in range(len(robots)):
                self.status_monitor[i].append(robots[i].status == 'BUSY')
                status_monitor_y = self.status_monitor[i]
                status_monitor_x = np.arange(len(status_monitor_y))
                number_agvs = len(self.global_robot_list.items)
                plt.subplot2grid((2, number_agvs + 3), (0, robots[i].ID + 2))
                plt.title("AGV" + str(robots[i].ID))
                plt.xlabel("Simulation time (s)")
                plt.ylabel("IDLE (0), BUSY (1)")
                plt.plot(status_monitor_x, status_monitor_y, 'b')

    def battery_status_monitor_update(self):
        while True:
            yield self.env.timeout(1)
            robots = self.global_robot_list.items
            robots = sorted(robots, key=lambda robot: robot.ID)
            for i in range(len(robots)):
                self.battery_status_monitor[i].append(robots[i].battery_status)
                battery_status_monitor_y = self.battery_status_monitor[i]
                battery_status_monitor_x = np.arange(len(battery_status_monitor_y))
                number_agvs = len(self.global_robot_list.items)
                plt.subplot2grid((2, number_agvs + 3), (1, robots[i].ID + 2))
                plt.xlabel("Simulation time (s)")
                plt.ylabel("Battery level")
                plt.ylim(0, 100)
                plt.plot(battery_status_monitor_x, battery_status_monitor_y, 'b')
                plt.plot(battery_status_monitor_x, np.repeat(20, len(battery_status_monitor_x)), 'r')

    def logging(self):

        # Create loggers
        log_filename_1 = 'global_task_list.log'
        log_filename_2 = 'local_task_lists.log'
        log_filename_3 = 'robot_paths.log'
        log_filename_4 = 'tasks_executing.log'

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
            log1.debug("At simulation time: " + str(self.env.now) + ":\t" + str(
                [task.to_string() for task in self.global_task_list.items]))
            robots = np.copy(self.global_robot_list.items)
            robots = sorted(robots, key=lambda robot: robot.ID)
            for i in range(len(self.local_task_lists)):
                log2.debug("At simulation time: " + str(self.env.now) + ":\t" + "robot " + str(i + 1) + "\t" + str(
                    [task.to_string() for task in self.local_task_lists[i].items]))
            for i in range(len(robots)):
                if robots[i].path:
                    log3.debug("At simulation time: " + str(self.env.now) + ":\t" + "robot " + str(i + 1) + "\t" + str(
                        [node.to_string() for node in robots[i].path]))
            log4.debug("At simulation time: " + str(self.env.now) + ":\t" + str(
                [task.to_string() for task in self.tasks_executing.items]))
