# AGV Simulator for students

This library contains a simulation setup for an AGV system of an infinite amount of AGVs in a predefined layout.

## Structure

- The logfiles folder contains logfiles of all the lists saved in the global database
    -   Global robot list keeps a list of all robots in the system with all their variables
    -   Global task list keeps a list of all tasks not yet executed and thus to be distributed
    -   Local task lists keeps a list of all the local task lists of each robot, a local task lists holds all tasks assigned to a robot but not yet executed
    -   Tasks executing holds a list of all tasks currently executing by a robot

- The scripts folder contains all executable scripts
    -   'generate_situation' generates a new 'orders_txt'-file
    -   'run_simulation' runs the simulation with a chosen order list and a specified number of robots

- The src folder contains all source code of the simulator
    -   'agv' contains all classes for the agv intelligence
        -   'AGV_Action' takes care of the agv's actions like moving
        -   'AGV_Comm' takes care of the agv's communication with FleetManager
        -   'AGV_Main' implements the 'dumb' main thread of the agv accepting tasks from local task list and executing them
        -   'AGV_ResourceManagement' implements the battery management behavior
        -   'AGV_TaskAllocation' implements the task allocation behavior central and decentral
    -   'datatypes' contains some datatypes for the simulator as 'Task' and 'Robot'
    -   'fleetmanagers' contains all FleetManager types possible, all trying to make a good task allocation
    -   'solvers' contains some used solvers like the popular Astar shortest path solver
    -   'utils' contains some basic functions like distance calculation
    -   'Graph' implements the factory layout
    -   'Logger' takes care of logging data to the logfiles
    -   'MES.py' implements the basic MES behavior, processing the order list and spawning the tasks to the FleetManager
    -   'RendererOnline' takes care of rendering the scene  during simulation
    -   'Simulation' implements the main simulation, setting up all entities (MES, FleetManager, AGV, database, ...)

- The test_vectors folder contains the simulation setup file and the order files needed for the simulation