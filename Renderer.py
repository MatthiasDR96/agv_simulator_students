import matplotlib.pyplot as plt
import math


class Renderer:

    def __init__(self, env, locations, global_task_list, global_robot_list):

        # Attributes
        self.env = env
        self.locations = locations
        self.global_robot_list = global_robot_list
        self.global_task_list = global_task_list

        # Processes
        self.render_scene = self.env.process(self.render_scene())

    def render_scene(self):

        while True:

            yield self.env.timeout(1)

            # Plot Graph
            plt.figure(1)
            for node in self.locations:
                plt.plot(node.pos[0], node.pos[1], 'b.', ms=6)
                plt.text(node.pos[0], node.pos[1]+0.1, node.name)
                for neighbor in node.neighbors:
                    plt.plot([node.pos[0], neighbor.pos[0]], [node.pos[1], neighbor.pos[1]], 'b-', lw=0.5)

            # Plot AGVs
            for robot in self.global_robot_list.items:
                plt.plot(robot.position[0], robot.position[1], 'ro', ms=10)
                plt.arrow(robot.position[0], robot.position[1], math.cos(robot.heading_direction), math.sin(robot.heading_direction), width=0.3, color='r')
                plt.text(robot.position[0], robot.position[1]+0.1, str("AGV" + str(robot.ID)))

            # Plot Tasks
            [plt.plot(task.pos_A[0], task.pos_A[1], 'gs', ms=7) for task in self.global_task_list.items]

            # Plot
            plt.draw()
            plt.pause(0.0001)
            plt.clf()

