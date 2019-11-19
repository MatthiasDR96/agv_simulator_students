class Task:

    def __init__(self, order_number, pos_a, pos_b, priority):

        self.order_number = order_number
        self.pos_A = pos_a
        self.pos_B = pos_b
        self.priority = priority
        self.robot = None
        self.picked = False

    def to_log(self):

        if not self.robot:
            return str(self.order_number) + ";" + str(self.pos_A[0]) + ';' + str(self.pos_A[1]) + ";" \
                   + str(self.pos_B[0]) + ';' + str(self.pos_B[1]) + ";" + "R" + str(self.robot) + ";" \
                   + str(self.priority) + ";" + str(int(self.picked))
        else:
            return str(self.order_number) + ";" + str(self.pos_A[0]) + ';' + str(self.pos_A[1]) + ";" \
                   + str(self.pos_B[0]) + ';' + str(self.pos_B[1]) + ";" + "R" + str(self.robot) + ";" \
                   + str(self.priority) + ';' + str(int(self.picked))

    def to_string(self):

        return '[' + str(self.order_number) + ", " + str(self.pos_A) + ", " + str(self.pos_B) + ']'
