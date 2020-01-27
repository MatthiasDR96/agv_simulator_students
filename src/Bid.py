class Bid:
    """
            A class containing the Task representation
    """
    
    def __init__(self):
        self.task = None
        self.value = None
        self.robot = None
    
    def to_log(self):
        return str(self.task.to_log()) + ";" + str(self.value) + ";" + str(self.robot.to_log())
    
    def to_string(self):
        return '[' + str(self.task.to_string()) + ", " + str(self.value) + ", " + str(self.robot.to_string) + ']'
