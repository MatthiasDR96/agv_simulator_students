class ResourceManagement:
    
    def __init__(self):
        self.a = None
    
    def compute_optimal_insertion(self, optimized_tour):
        return None
    
    def charge(self):
        # Search for closest charging station
        self.status = 'CHARGING'
        print("AGV " + str(self.ID) + ":      Goes to closest charging station at " + str(self.env.now))
        closest_charging_station = self.search_closest_charging_station()
        
        # Go to the charging station
        yield self.env.process(self.execute_path(closest_charging_station))
        
        # Compute amount to charge
        tmp_scale = 0.01
        amount_to_charge = 100 - self.battery_status
        charge_time = ((self.max_charging_time * amount_to_charge) / 100) * tmp_scale
        print("AGV " + str(self.ID) + ":      Is charging for " + str(charge_time) + " seconds at " + str(
            self.env.now))
        
        # Charge
        yield self.env.timeout(charge_time)
        self.battery_status = 100
        self.status = 'IDLE'
