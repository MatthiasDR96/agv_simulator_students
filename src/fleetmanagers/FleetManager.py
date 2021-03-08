from src.agv.AGV_Comm import Comm
from src.utils.utils import *


class FleetManager:
    """
            A class containing the intelligence of the Fleetmanager agent inheriting a simple task allocation,
            allocating the task to the closest available robot. Robots have no local task list.
    """

    def __init__(self, env, kb, agv_fm_comm, agv_to_fm_comm, print_):

        # Simulation environment
        self.env = env

        # Printing
        self.print = print_

        # Communication attributes
        self.ip = '172.21.0.0'
        self.kb = kb
        self.agv_fm_comm = agv_fm_comm
        self.agv_to_fm_comm = agv_to_fm_comm
        self.comm = Comm(self.ip)

        # Process
        self.main = self.env.process(self.main())

        # Initialize
        self.my_print("Fleet manager:           Started")

    def main(self):

        self.my_print("\n")
        while True:

            # Timeout
            yield self.env.timeout(1)  # Sample time

            # Define current status
            idle_robots = self.get_idle_robots()
            tasks_to_assign = self.get_tasks_to_assign()

            # If there are idle robots and tasks to execute, assign closest task
            if not len(tasks_to_assign) == 0 and not len(idle_robots) == 0:
                # Pick first task
                task = tasks_to_assign[0]

                # Get closest robot
                robot = self.get_closest_robot(task, idle_robots)

                # Assign task to agv task lists
                task.message = 'push'
                self.comm.tcp_write(self.agv_fm_comm[robot.ID], task)

                # Remove task from global task list
                self.comm.sql_remove_task(self.kb['global_task_list'], task.order_number)

    def get_idle_robots(self):
        robots = np.copy(self.comm.sql_read(self.kb['global_robot_list']))
        robots = list(filter(lambda robot: robot.status == 'IDLE', robots))
        robots = sorted(robots, key=lambda robot_: robot_.ID)
        return robots

    def get_tasks_to_assign(self):
        tasks = np.copy(self.comm.sql_read(self.kb['global_task_list']))
        return tasks

    def get_closest_robot(self, task, idle_robots):
        return min(idle_robots,
                   key=lambda robot: calculate_astar_distance(self.kb['graph'], robot.robot_node, task.pos_A))

    def my_print(self, msg):
        if self.print:
            print(msg)
