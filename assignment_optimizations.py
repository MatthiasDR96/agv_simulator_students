import numpy as np
from random import randint

iteration = 100000
number_robots = 5
number_tasks = 7
distance_matrix = np.random.rand(number_robots, number_tasks)

best_fitness = 10000000
best_solution = []
for i in range(iteration):

    # Make solution
    x = np.zeros((number_robots, number_tasks))
    for j in range(number_tasks):
        k = randint(0, number_robots-1)
        x[k][j] = 1

    # Calculate fitness
    fitness = np.sum(np.multiply(x, distance_matrix))

    # Update
    if fitness < best_fitness:
        best_fitness = fitness
        best_solution = x

print(best_fitness)
print(best_solution)
