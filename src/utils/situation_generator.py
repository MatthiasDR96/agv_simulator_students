import ast
import configparser
from random import *

import numpy as np


def generate_situation(num_tasks, setup_file, order_list_file):
    # Setup
    setup = configparser.ConfigParser()
    setup.read(setup_file)

    # Set params
    max_arrival_interval = int(setup['SITUATION_GENERATOR']['max_arrival_interval'])
    priorities = ast.literal_eval(setup['SITUATION_GENERATOR']['priorities'])
    task_locations = ast.literal_eval(setup['LAYOUT']['task_locations'])

    # Init
    order_list = []
    order_numbers = []
    entering_time = 0

    # Shuffle task locations
    index_list = np.arange(0, len(task_locations))
    shuffle(index_list)

    for i in range(num_tasks):

        # New order
        order = list()

        # Generate entering time
        entering_time += randint(0, max_arrival_interval)
        order.append(entering_time)

        # Generate random order number of four digits
        order_number = randint(0, 9) * 1000 + randint(0, 9) * 100 + randint(0, 9) * 10 + randint(0, 9)
        while order_number in order_numbers:
            order_number = randint(0, 9) * 1000 + randint(0, 9) * 100 + randint(0, 9) * 10 + randint(0, 9)
        order.append(order_number)

        # Generate random task priority
        priority = priorities[randint(0, 2)]
        order.append(priority)

        # Generate two task locations from the layout
        pos_a = task_locations[index_list[i]]
        order.append(pos_a)

        # Append order
        order_list.append(order)

    # Save situation
    save_situation(order_list, order_list_file)

    # Print
    print("\nOrder list with " + str(num_tasks) + " tasks created in file '" + str(order_list_file) + "'.")


def save_situation(order_list, order_list_file):
    file1 = open(order_list_file, "w")
    for order in order_list:
        file1.write(str(order[0]) + ",")
        file1.write(str(order[1]) + ",")
        file1.write(str(order[2]) + ",")
        file1.write(str(order[3]) + ",\n")
    file1.close()
