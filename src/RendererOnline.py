import math

import matplotlib
import matplotlib.pyplot as plt
import numpy as np


class RendererOnline:
    
    def __init__(self, env, kb, depot_locations, charge_locations):

        # Attributes
        self.env = env
        self.locations = kb['graph']
        self.global_robot_list = kb['global_robot_list']
        self.global_task_list = kb['global_task_list']
        self.status_monitor = [[] for robot in kb['global_robot_list'].items]
        self.battery_status_monitor = [[] for robot in kb['global_robot_list'].items]
        self.tasks_executing = kb['tasks_executing']
        self.depot_locations = depot_locations
        self.charge_locations = charge_locations
        self.tasks_not_executing_monitor = []
        self.colors = matplotlib.cm.rainbow(np.linspace(0, 1, len(kb['global_robot_list'].items)))

        # Processes
        self.render_scene = self.env.process(self.render_scene())
        self.status_monitor_update = self.env.process(self.status_monitor_update())
        self.battery_status_monitor_update = self.env.process(self.battery_status_monitor_update())
        # self.tasks_not_executing_monitor_update = self.env.process(self.tasks_not_executing_monitor_update())

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

                # Plot path
                if robot.path:
                    plt.plot([robot.position[0], robot.path[0].pos[0]], [robot.position[1], robot.path[0].pos[1]],
                             color=color, lw=1.0)
                    for i in range(len(robot.path)):
                        plt.plot(robot.path[i].pos[0], robot.path[i].pos[1], color=color, marker='.', ms=10)
                        if i < len(robot.path) - 1:
                            plt.plot([robot.path[i].pos[0], robot.path[i + 1].pos[0]],
                                     [robot.path[i].pos[1], robot.path[i + 1].pos[1]], color=color, lw=1.5)

                # Plot assigned tasks
                # list = self.local_task_lists[robot.ID - 1]
                # for item in list.items:
                # plt.plot([robot.position[0], item.pos_A[0]],
                # [robot.position[1], item.pos_A[1]], color=color, lw=0.5)

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
    
    # def tasks_not_executing_monitor_update(self):
    
    # while True:
    # yield self.env.timeout(1)
    # tasks_not_executing = np.sum(
    # [len(self.local_task_lists[i].items) for i in range(len(self.global_robot_list.items))])
    # self.tasks_not_executing_monitor.append(tasks_not_executing)
    # monitor_tasks_not_executing_y = self.tasks_not_executing_monitor
    # monitor_tasks_not_executing_x = np.arange(len(monitor_tasks_not_executing_y))
    # plt.figure(3)
    # plt.title("Tasks not executing")
    # plt.ylabel("Amount of tasks not executing")
    # plt.xlabel("Simulation time (s)")
    # plt.plot(monitor_tasks_not_executing_x, monitor_tasks_not_executing_y)
    # plt.draw()
    #plt.pause(0.0001)
