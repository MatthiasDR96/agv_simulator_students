class Task:

    def __init__(self, order_number, pos_a, pos_b):

        self.order_number = order_number
        self.pos_A = pos_a
        self.pos_B = pos_b

    def to_string(self):

        return "[" + str(self.order_number) + ", " + str((self.pos_A[0], self.pos_A[1])) + ", " + str((self.pos_B[0], self.pos_B[1])) + "]"
