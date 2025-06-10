class Controller:
    def __init__(self):
        self.kp = 1.
        self.ki = 0.
        self.kd = 0.

    def step(self, inputParameters):
        outputParameters = {'theta_x': 0.}
        outputParameters['theta_x'] = inputParameters['a_x'] * 0.1
        return outputParameters
