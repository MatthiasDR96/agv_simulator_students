class Robot:

    def __init__(self, id_number, position, heading_direction=0, path=None, status='IDLE', battery_status=100):
        self.ID = id_number
        self.position = position
        self.heading_direction = heading_direction
        self.path = path
        self.status = status
        self.battery_status = battery_status

    def to_string(self):
        return "[" + str(self.ID) + ", " + str(self.position) + "]"
