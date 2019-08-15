import numpy as np
from ortools.graph import pywrapgraph

number_robots = 5
number_tasks = 5
cost = np.random.rand(number_robots, number_tasks) * 10
cost = cost.astype(int)


def main():
    rows = len(cost)
    cols = len(cost[0])

    assignment = pywrapgraph.LinearSumAssignment()

    for worker in range(rows):
        for task in range(cols):
            if cost[worker][task]:
                assignment.AddArcWithCost(worker, task, cost[worker][task])

    solve_status = assignment.Solve()

    if solve_status == assignment.OPTIMAL:
        print('Total cost = ', assignment.OptimalCost())
        print()
        for i in range(0, assignment.NumNodes()):
            print('Worker %d assigned to task %d.  Cost = %d' % (
                i,
                assignment.RightMate(i),
                assignment.AssignmentCost(i)))
    elif solve_status == assignment.INFEASIBLE:
        print('No assignment is possible.')
    elif solve_status == assignment.POSSIBLE_OVERFLOW:
        print('Some input costs are too large and may cause an integer overflow.')


if __name__ == "__main__":
    main()
