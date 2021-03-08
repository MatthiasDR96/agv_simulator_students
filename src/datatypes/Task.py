class Task:
    """
            A class containing the Task representation
    """

    def __init__(self, order_number, pos_a, priority=0):
        self.order_number = order_number
        self.pos_A = pos_a
        self.priority = priority
        self.robot = None
        self.picked = False
        self.message = None

    def to_log(self):
        return str(self.order_number) + ";" + str(self.pos_A) + \
               ";" + "R" + str(self.robot) + ";" + str(self.priority) + ";" + str(int(self.picked))

    def to_string(self):
        return '[' + str(self.order_number) + ", " + str(self.pos_A) + ", " + str(self.priority) + ']'
