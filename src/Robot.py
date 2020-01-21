class Robot:
    """
            A class containing the Robot representation
    """
    
    def __init__(self, id_number, position, heading_direction=0, path=None, status='IDLE', battery_status=100):
        self.ID = id_number
        self.position = position
        self.heading_direction = heading_direction
        self.path = path
        self.status = status
        self.battery_status = battery_status

    def to_log(self):
        if self.path:
            path_string = ""
            for node in self.path:
                path_string += node.to_string() + ','
        else:
            path_string = ""
        return str(self.ID) + ";" + str(self.position[0]) + ";" + str(self.position[1]) + ";" + str(
            self.status) + ";" + str(self.battery_status) \
               + ";" + str(self.heading_direction) + ";" + path_string

    def to_string(self):
        return '[' + str(self.ID) + ", " + str(self.position) + ", " + str(self.status) + ", " + \
               str(self.battery_status) + ']'
