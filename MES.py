from random import randint

from Task import Task


class MES:

    def __init__(self, env, global_task_list, task_locations, amount_of_tasks, max_time_interval, tasks_executing):

        # Attributes
        self.env = env
        self.global_task_list = global_task_list  # MES writes new tasks to this list
        self.task_locations = task_locations  # Locations where tasks can be spawned
        self.amount_of_tasks = amount_of_tasks  # Amount of tasks which needs to be done in the simulation
        self.max_time_interval = max_time_interval  # Max interval between the task arrivals
        self.priorities = [1, 2, 3]  # Task priorities
        self.tasks_executing = tasks_executing

        # Processes
        self.main = self.env.process(self.main())

        # Initialize
        print("\nMES:              Started")

    def main(self):

        for i in range(self.amount_of_tasks):

            # Random time interval between task announcements
            yield self.env.timeout(randint(0, self.max_time_interval))

            # Generate random order number of four digits
            order_number = randint(0, 9) * 1000 + randint(0, 9) * 100 + randint(0, 9) * 10 + randint(0, 9)

            # Generate random task priority
            priority = self.priorities[randint(0, 2)]

            # Generate two random task locations from the layout
            pos_a = self.task_locations[randint(0, len(self.task_locations) - 1)]
            pos_b = self.task_locations[randint(0, len(self.task_locations) - 1)]

            # Look if the order number is already used in global task list
            same_order_number = False
            for task in self.global_task_list.items:
                if task.order_number == order_number:
                    same_order_number = True
                    break

            # If pick-up and drop-off locations are different, and the order number is not used already, create task
            if (not (pos_a[0] == pos_b[0] and pos_a[1] == pos_b[1])) and not same_order_number:
                # Create task instance
                new_task = Task(order_number, pos_a, pos_b, priority)

                # Put the new task in the global task list
                self.global_task_list.put(new_task)
                print('MES:              New task ' + new_task.to_string() + ' arrived at ' + str(self.env.now))

        while True:

            yield self.env.timeout(1)
            if len(self.tasks_executing.items) == 0 and len(self.global_task_list.items) == 0:
                print("\nEnd of simulation at " + str(self.env.now))
                break
