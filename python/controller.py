class Controller:
    def __init__(self):
        self.kp = 1.
        self.ki = 0.
        self.kd = 0.
    def calculate_control(self, a, a_dot):
        theta = 0.
        return theta