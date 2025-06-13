import numpy as np
class Controller:
    def __init__(self, TP):
        self.dt = TP
        self.kp = 0.1
        self.kd = 0

        self.last_a = 0.
        self.last_theta = 0.

    def step(self, a, a_dot):
        # sign = 1 if a > 0 else -1

        theta = a * self.kp

        # theta *= sign

        theta = self.rate_limiter(theta, self.last_theta, 1)
        self.last_theta = theta

        return theta

    def rate_limiter(self, x, x_last, max_rate):
        delta = x - x_last
        max_delta = max_rate * self.dt
        delta_limited = max(-max_delta, min(delta, max_delta))
        return x_last + delta_limited

    def iir(self, x, last_x, alpha):
        y = alpha * x + (1 - alpha) * last_x
        return y

    def derive(self, var, lastVar):
        return (var - lastVar) / self.dt
