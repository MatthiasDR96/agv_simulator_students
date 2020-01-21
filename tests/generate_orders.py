from random import *

import numpy as np

"""
        Script to generate an orders.txt file used as an input to the simulation
"""

# Init params
amount_of_tasks_to_execute = 30
max_arrival_interval = 20  # sec
priorities = [1, 2, 3]

# Define graph
node_locations = [(10, 20), (10, 40), (10, 60), (20, 0), (20, 20), (20, 40), (20, 60), (20, 80), (30, 0), (30, 20),
                  (30, 40), (30, 60), (30, 80)]
task_locations = np.delete(node_locations, 1, axis=0)  # No tasks at depot location and charging locations
order_list = []
entering_time = 0

# Open file
file1 = open("../orders_3.txt", "w")

for i in range(amount_of_tasks_to_execute):

    # Generate random order number of four digits
    order_number = randint(0, 9) * 1000 + randint(0, 9) * 100 + randint(0, 9) * 10 + randint(0, 9)

    # Generate random task priority
    priority = priorities[randint(0, 2)]

    # Generate entering time
    entering_time += randint(0, max_arrival_interval)

    # Generate two random task locations from the layout
    pos_a = task_locations[randint(0, len(task_locations) - 1)]
    pos_b = task_locations[randint(0, len(task_locations) - 1)]

    # Look if the order number is already used in global task list
    same_order_number = False
    for order in order_list:
        if order == order_number:
            same_order_number = True
            break

    # If pick-up and drop-off locations are different, and the order number is not used already, create task
    if (not (pos_a[0] == pos_b[0] and pos_a[1] == pos_b[1])) and not same_order_number:
        # Write task to file
        order_list.append(order_number)
        file1.write(str(entering_time) + ",")
        file1.write(str(order_number) + ",")
        file1.write(str(priority) + ",")
        file1.write(str(pos_a[0]) + ",")
        file1.write(str(pos_a[1]) + ",")
        file1.write(str(pos_b[0]) + ",")
        file1.write(str(pos_b[1]) + ",\n")

file1.close()
