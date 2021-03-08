from src.utils.situation_generator import *

# Set params
setup_file = '../test_vectors/setup.ini'
order_list_file = '../test_vectors/orders.txt'

# Number of tasks
num_tasks = 10

# Generate situation
generate_situation(num_tasks, setup_file, order_list_file)
