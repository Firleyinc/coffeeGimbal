import numpy as np
from matplotlib.pyplot import thetagrids


class LowPassFilter:
    def __init__(self, alpha, initial=0.0):
        self.alpha = alpha
        self.value = initial

    def step(self, new_value):
        self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value

class Controller:
    def __init__(self, TP):
        self.dt = TP
        self.kp = 1.
        self.kd = 0

        self.last_a = 0.
        self.last_theta = 0.

        self.a_lpf = LowPassFilter(alpha=0.2)

    def step(self, a, a_dot):
        # theta = a * 0.1
        # a = self.a_lpf.step(a)

        theta = -self.calc_netVect_angle(a, -9.81) * self.kp

        theta = self.rate_limiter(theta, self.last_theta, 1)
        self.last_theta = theta

        return theta

    def calc_netVect_angle(self, a, g):
        alfa = np.arctan(a / g)
        return alfa

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
