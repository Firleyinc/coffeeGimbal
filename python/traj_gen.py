import numpy as np

class TrajGen:
    def __init__(self, freq=1., amplitude=1.):
        self.maxVal = amplitude/2
        self.minVal = -self.maxVal
        self.period = 1/freq


    def sine(self, time, phase=0, freq=None):
        if freq is not None:
            self.period = 1/freq
        val = np.sin(2 * np.pi * time / self.period + phase)
        scaled_val = self.minVal + (val + 1) / 2 * (self.maxVal - self.minVal)
        return scaled_val

