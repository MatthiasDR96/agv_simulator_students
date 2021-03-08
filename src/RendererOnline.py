import math

import matplotlib
import matplotlib.pyplot as plt
import numpy as np


class RendererOnline:
    """
            A class containing the methods for an online rendering of the simulation.
    """

    def __init__(self, env, kb, depot_locations, charge_locations, print_):

        # Printing
        self.print = print_

        # Attributes
        self.env = env
        self.kb = kb
        self.graph = kb['graph']
        self.global_robot_list = kb['global_robot_list']
        self.global_task_list = kb['global_task_list']
        self.status_monitor = [[] for _ in kb['global_robot_list'].items]
        self.battery_status_monitor = [[] for _ in kb['global_robot_list'].items]
        self.tasks_executing = kb['tasks_executing']
        self.depot_locations = depot_locations
        self.charge_locations = charge_locations
        self.tasks_not_executing_monitor = []
        self.colors = matplotlib.cm.rainbow(np.linspace(0, 1, len(kb['global_robot_list'].items)))

        # Processes
        self.render_scene = self.env.process(self.render_scene())
        self.status_monitor_update = self.env.process(self.status_monitor_update())
        self.battery_status_monitor_update = self.env.process(self.battery_status_monitor_update())

        # Initiate plt
        plt.close("all")
        plt.rcParams['toolbar'] = 'None'
        plt.figure("agv Simulator", figsize=(11, 6), facecolor='k', edgecolor='k', dpi=100)
        plt.title("agv Simulator")

    def render_scene(self):

        while True:

            yield self.env.timeout(1)
            number_agvs = len(self.global_robot_list.items)
            ax = plt.subplot2grid((1, number_agvs + 3), (0, 0), colspan=3)
            plt.subplots_adjust(wspace=1.5, hspace=0.4)
            plt.title("agv Layout")
            plt.style.use('dark_background')

            # Plot Graph
            self.graph.plot(ax)

            # Plot AGVs
            for robot in self.global_robot_list.items:
                color = self.colors[robot.ID - 1]
                plt.plot(robot.robot_location[0], robot.robot_location[1], color=color, marker='o', ms=10)
                plt.arrow(robot.robot_location[0], robot.robot_location[1], math.cos(robot.heading_direction),
                          math.sin(robot.heading_direction), width=0.3, color=color)
                plt.text(robot.robot_location[0] + 0.5, robot.robot_location[1] + 1.5, str("agv" + str(robot.ID)))

                # Plot path
                if robot.path:
                    node_pos = self.kb['graph'].nodes[robot.path[0]].pos
                    plt.plot([robot.robot_location[0], node_pos[0]], [robot.robot_location[1], node_pos[1]],
                             color=color, lw=3.5)
                    for i in range(len(robot.path)):
                        node_pos = self.kb['graph'].nodes[robot.path[i]].pos
                        plt.plot(node_pos[0], node_pos[1], color=color, marker='.', ms=15)
                        if i < len(robot.path) - 1:
                            node_pos_ = self.kb['graph'].nodes[robot.path[i + 1]].pos
                            plt.plot([node_pos[0], node_pos_[0]],
                                     [node_pos[1], node_pos_[1]], color=color, lw=3.5)

                # Plot total path
                if robot.total_path:
                    for i in range(len(robot.total_path)):
                        node_pos = self.kb['graph'].nodes[robot.total_path[i]].pos
                        plt.plot(node_pos[0], node_pos[1], color=color, marker='.', ms=15)
                        if i < len(robot.total_path) - 1:
                            node_pos_ = self.kb['graph'].nodes[robot.total_path[i + 1]].pos
                            plt.plot([node_pos[0], node_pos_[0]],
                                     [node_pos[1], node_pos_[1]], color=color, lw=1.5)

                # Plot assigned tasks
                for key in self.kb.keys():
                    if key == 'local_task_list_R' + str(robot.ID):
                        list_ = self.kb[key]
                        for item in list_.items:
                            node_pos = self.kb['graph'].nodes[item.pos_A].pos
                            plt.plot(node_pos[0], node_pos[1], 'bs', ms=7)
                            plt.text(node_pos[0] - 1, node_pos[1] + 1.5, str(item.order_number))
                            plt.plot([robot.robot_location[0], node_pos[0]],
                                     [robot.robot_location[1], node_pos[1]], color=color, lw=0.5)

            # Plot tasks executing
            for task in self.tasks_executing.items:
                node_pos = self.kb['graph'].nodes[task.pos_A].pos
                plt.plot(node_pos[0], node_pos[1], 'rs', ms=7)
                plt.text(node_pos[0] - 1, node_pos[1] + 1.5, str(task.order_number))

            # Plot tasks in global task list
            for task in self.global_task_list.items:
                node_pos = self.kb['graph'].nodes[task.pos_A].pos
                plt.plot(node_pos[0], node_pos[1], 'gs', ms=7)
                plt.text(node_pos[0] - 1, node_pos[1] + 1.5, str(task.order_number))

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
                plt.title("agv" + str(robots[i].ID))
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

    def my_print(self, msg):
        if self.print:
            print(msg)
