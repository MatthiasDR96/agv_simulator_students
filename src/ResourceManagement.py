class ResourceManagement:
    """
            A class containing the intelligence of the Resource Management agent
    """
    
    def __init__(self, agv):
        
        # AGV
        self.agv = agv
    
    # Classic charging scheme, charge when threshold reached, head to closest charging station, and charge fully
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
    
    def search_closest_charging_station(self, location):
        closest_point = None
        min_distance = 1000
        for position in self.agv.charging_stations:
            distance, _ = self.agv.astar.find_shortest_path(position, location)
            if distance < min_distance:
                min_distance = distance
                closest_point = position
        return closest_point
