from ResourceOptimizer import ResourceOptimizer


class ResourceManagement:
    """
            A class containing the intelligence of the Resource Management agent
    """
    
    def __init__(self, agv):
        
        # AGV
        self.agv = agv

        # Particle swarm
        self.ropt = ResourceOptimizer(self.agv)
    
    def charge_classic(self):
        # Search for closest charging station
        self.agv.status = 'CHARGING'
        print("AGV " + str(self.agv.ID) + ":      Goes to closest charging station at " + str(self.agv.env.now))
        closest_charging_station = self.search_closest_charging_station(self.agv.robot_location)
        
        # Go to the charging station
        yield self.agv.env.process(self.agv.execute_path(closest_charging_station))
        
        # Compute amount to charge
        tmp_scale = 0.01
        amount_to_charge = 100 - self.agv.battery_status
        charge_time = ((self.agv.max_charging_time * amount_to_charge) / 100) * tmp_scale
        print("AGV " + str(self.agv.ID) + ":      Is charging for " + str(charge_time) + " seconds at " + str(
            self.agv.env.now))
        
        # Charge
        yield self.agv.env.timeout(charge_time)
        self.agv.battery_status = 100
        self.agv.status = 'IDLE'
    
    def compute_optimal_insertion_and_charging_time(self, initial_tour, initial_resources):
    
        # Compute cost for robot location to first location in initial tour plus closest charging station after end
        closest_charging_station = self.search_closest_charging_station(initial_tour[-1].pos_B)
        if self.agv.task_executing:
            tour_to_evaluate = [self.agv.task_executing.pos_B] + initial_tour + [closest_charging_station]  # TODO tuple
        else:
            tour_to_evaluate = initial_tour + [closest_charging_station]
        travel_times, _, resource_at_city, total_resource_time = self.ropt.evaluate_tour(tour_to_evaluate, 0)
        self.ropt.plot_resource_at_city(resource_at_city)

        # If no charging needed, return initial tour, otherwise insert charging station
        if resource_at_city[-1] > self.ropt.minimum_resources:
            print("\nNo need to optimize")
            optimal_tour, optimal_charging_time, fitness = initial_tour, 0, sum(travel_times) + total_resource_time
        else:
            print("\nNeed to optimize")
            optimal_tour, optimal_charging_time, fitness = self.ropt.optimize_brute_force(initial_tour, initial_resources)
        
        return optimal_tour, optimal_charging_time, fitness
    
    def search_closest_charging_station(self, location):
        closest_point = None
        min_distance = 1000
        for position in self.agv.charging_stations:
            distance, _ = self.agv.astar.find_shortest_path(position, location)
            if distance < min_distance:
                min_distance = distance
                closest_point = position
        return closest_point
