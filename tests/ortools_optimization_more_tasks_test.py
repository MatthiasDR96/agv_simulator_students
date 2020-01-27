from __future__ import print_function

import numpy as np
from ortools.sat.python import cp_model


def main():
    model = cp_model.CpModel()

    cost = [[90, 76, 75, 70, 50, 74, 12, 68],
            [35, 85, 55, 65, 48, 101, 70, 83],
            [125, 95, 90, 105, 59, 120, 36, 73]]
    # [45, 110, 95, 115, 104, 83, 37, 71],
    # [60, 105, 80, 75, 59, 62, 93, 88],
    # [45, 65, 110, 95, 47, 31, 81, 34],
    # [38, 51, 107, 41, 69, 99, 115, 48],
    # [47, 85, 57, 71, 92, 77, 109, 36],
    # [39, 63, 97, 49, 118, 56, 92, 61],
    # [47, 101, 71, 60, 88, 109, 52, 90]]

    sizes = [10, 7, 3, 12, 15, 4, 11, 5]
    total_size_max = 15
    num_workers = len(cost)
    num_tasks = len(cost[1])

    # Variable matrix x
    x = []
    for i in range(num_workers):
        t = []
        for j in range(num_tasks):
            t.append(model.NewIntVar(0, 1, "x[%i,%i]" % (i, j)))
        x.append(t)

    # Constraints
    # Each task is assigned to exact one worker.
    [model.Add(sum(x[i][j] for i in range(num_workers)) == 1)
     for j in range(num_tasks)]

    # Solve
    model.Minimize(sum([np.dot(x_row, cost_row) for (x_row, cost_row) in zip(x, cost)]))
    solver = cp_model.CpSolver()
    status = solver.Solve(model)

    if status == cp_model.OPTIMAL:
        print('Minimum cost = %i' % solver.ObjectiveValue())

        for i in range(num_workers):
            for j in range(num_tasks):
    
                if solver.Value(x[i][j]) == 1:
                    print('Worker ', i, ' assigned to task ', j, '  Cost = ', cost[i][j])
        print()


if __name__ == '__main__':
    main()
