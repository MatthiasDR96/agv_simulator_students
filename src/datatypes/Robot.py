class Robot:
    """
            A class containing the Robot representation
    """

    def __init__(self, id_number, robot_location, robot_node, heading_direction=0, path=None, status='IDLE',
                 battery_status=100,
                 travelled_time=0, charged_time=0, congestions=0, task_executing=None, total_path=None):

        self.ID = id_number
        self.robot_location = robot_location
        self.robot_node = robot_node
        self.status = status
        self.battery_status = battery_status
        self.travelled_time = travelled_time
        self.charged_time = charged_time
        self.heading_direction = heading_direction
        self.task_executing = task_executing
        self.path = path
        self.congestions = congestions
        self.total_path = total_path

    def to_log(self):
        if self.path:
            path_string = "{"
            for node in self.path:
                path_string += node + ','
            path_string = path_string[:-1]
            path_string += "}"
        else:
            path_string = "{}"
        return str(self.ID) + ";" + str(self.robot_location[0]) + ";" + str(self.robot_location[1]) + ";" + str(
            self.status) + ";" + str(self.battery_status) \
               + ";" + str(self.heading_direction) + ";" + path_string + ";" + str(self.travelled_time) \
               + ";" + str(self.charged_time) + ";" + str(self.congestions)

    def to_string(self):
        return '[' + str(self.ID) + ", " + str(self.robot_location) + ", " + str(self.status) + ", " + \
               str(self.battery_status) + ']'
