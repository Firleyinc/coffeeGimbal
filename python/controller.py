import numpy as np
class Controller:
    def __init__(self):
        self.kp = 1.
        self.ki = 0.
        self.kd = 0.

    def step(self, inputParameters):
        outputParameters = {'theta_x': 0.}
        outputParameters['theta_x'] = np.clip(inputParameters['a_x'] * 0.1, -5., 5.)
        return outputParameters
