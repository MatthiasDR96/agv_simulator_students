from Task import Task


class MES:

    def __init__(self, env, global_task_list, tasks_executing, order_list):

        # Attributes
        self.env = env  # Environment
        self.global_task_list = global_task_list  # MES writes new tasks to this list
        self.tasks_executing = tasks_executing  # Tasks which are still executing (for end criterium)
        self.order_list = order_list  # List of orders to spawn

        # Processes
        self.main = self.env.process(self.main())

        # Initialize
        print("\nMES:              Started")

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
            pos_a = (int(order[3]), int(order[4]))
            pos_b = (int(order[5]), int(order[6]))
            priority = int(order[2])
            new_task = Task(order_number, pos_a, pos_b, priority)

            # Put the new task in the global task list
            self.global_task_list.put(new_task)
            print('MES:              New task ' + new_task.to_string() + ' arrived at ' + str(self.env.now))

            # Read order
            line = file.readline()
            order = line.split(',')

        # Close file
        file.close()

        # Wait till all tasks are executed
        while True:
            yield self.env.timeout(1)
            if len(self.tasks_executing.items) == 0 and len(self.global_task_list.items) == 0:
                break
