from src.Simulation import Simulation

# Set params
setup_file = '../test_vectors/setup.ini'
orders_file = '../test_vectors/orders.txt'
number_of_robots = 3
render_ = False
log_ = True
print_ = True

# Create simulator
sim = Simulation(setup_file)

# Start simulation
sim_time, travel_cost, charging_cost, congestions = sim.start_simulation(orders_file, number_of_robots,
                                                                         render_=render_, print_=print_, log_=log_)

# Print output
print("\nSimulation ended: ")
print("\tSimulation time: " + str(sim_time) + ' seconds')
