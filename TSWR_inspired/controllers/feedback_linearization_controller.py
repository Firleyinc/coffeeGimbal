import numpy as np
from model.model_definition import GimbalModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = GimbalModel(Tp)

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """
        q1, q2, q1_dot, q2_dot = x

        q = np.array([q1,q2])
        q_dot = np.array([q1_dot,q2_dot])


        M = self.model.M(x)
        C = self.model.C(x)

        Kp = np.diag([20,20])
        Kd = np.diag([5,5])
        
        v = q_r_ddot# - Kd @ (q_dot - q_r_dot) - Kp @ (q-q_r)


        tau = M @ v + C @ q_dot
        
        return tau.flatten()
