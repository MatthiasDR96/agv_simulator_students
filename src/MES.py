from src.agv.AGV_Comm import Comm
from src.datatypes.Task import Task


class MES:

    """
            A class containing the intelligence of the MES agent
    """

    def __init__(self, env, kb, order_list, print_):

        # Simulation environment
        self.env = env

        # Printing
        self.print = print_

        # Communication attributes
        self.ip = '172.21.0.0'
        self.kb = kb  # Knowledge base
        self.comm = Comm(self.ip)

        # List of orders to spawn
        self.order_list = order_list

        # Process
        self.main = self.env.process(self.main())

        # Initialize
        self.my_print("\nMES:                     Started")

    def main(self):

        # Open file
        file = open(self.order_list, "r")

        # Read order
        line = file.readline()
        order = line.split(',')
        prev_time = 0
        while line != "":

            # Calculate spawn time and wait for this time
            execution_time = int(order[0])
            timeout = execution_time - prev_time
            prev_time = execution_time
            yield self.env.timeout(timeout)

            # Create new task
            order_number = int(order[1])
            priority = int(order[2])
            pos_a = order[3]
            new_task = Task(order_number, pos_a, priority)

            # Put the new task in the global task list
            self.comm.sql_write(self.kb['global_task_list'], new_task)
            self.my_print('MES: New task ' + new_task.to_string() + ' arrived at ' + str(self.env.now))

            # Read order
            line = file.readline()
            order = line.split(',')

        # Close file
        file.close()

        # Wait till all tasks are executed
        while True:

            # Sample time
            yield self.env.timeout(2)

            # Check amount of tasks in local task list
            amount_of_tasks_in_local_task_lists = 0
            for key in self.kb.keys():
                if 'local_task_list_R' in key:
                    amount_of_tasks_in_local_task_lists += len(self.kb[key].items)

            # Check amount of tasks in executing task list
            amount_of_executing_tasks = len(self.kb['tasks_executing'].items)

            # Check amount af tasks in global task list
            amount_of_global_tasks = len(self.kb['global_task_list'].items)

            # End criterium
            if amount_of_executing_tasks == 0 and amount_of_tasks_in_local_task_lists == 0 \
                    and amount_of_global_tasks == 0:
                break

    def my_print(self, msg):
        if self.print:
            print(msg)
