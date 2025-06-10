import numpy as np
from model.model_definition import GimbalModel
from model.model_definition2 import GimbalModel2
from model.model_definition3 import GimbalModel3
from .controller import Controller


class MMAController(Controller):
    def __init__(self, Tp):
        self.model1 = GimbalModel(Tp)
        self.model2 = GimbalModel2(Tp)
        self.model3 = GimbalModel3(Tp)
        self.models = [self.model1, self.model2, self.model3]
        self.i = 0

    def choose_model(self, x):
        errors = []
        self.em1 = np.linalg.norm(x-self.model1.predict(x))
        errors.append(self.em1)
        self.em2 = np.linalg.norm(x-self.model2.predict(x))
        errors.append(self.em2)
        self.em3 = np.linalg.norm(x-self.model3.predict(x))
        errors.append(self.em3)

        if np.argmin(errors) == self.em1:
            self.i = 0
        if np.argmin(errors) == self.em2:
            self.i = 1
        if np.argmin(errors) == self.em3:
            self.i = 2

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        q1, q2, q1_dot, q2_dot = x
        self.choose_model(x)
        q = x[:2]
        q_dot = x[2:]
        Kp = np.diag([20,20])
        Kd = np.diag([5,5])
        v = q_r_ddot - Kd @ (q_dot - q_r_dot) - Kp @ (q-q_r)
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v[:, np.newaxis] + C @ q_dot[:, np.newaxis]
        return u
