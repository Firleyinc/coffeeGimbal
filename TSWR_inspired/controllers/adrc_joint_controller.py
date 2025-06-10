import numpy as np
from observers.eso import ESO
from .controller import Controller
from model.model_definition import GimbalModel



class ADRCJointController(Controller):
    def __init__(self, kp, kd, p, q0, Tp, joint_index):
        self.b = 1.0
        self.kp = kp
        self.kd = kd

        self.model = GimbalModel(Tp)
        self.joint_index = joint_index

        self.l1 = 3 * p
        self.l2 = 3 * p**2
        self.l3 = p**3


        A = np.array([[0, 1.0 , 0],[0, 0, 1.0],[0, 0, 0]])
        B = [[0], [self.b], [0]] #estymowane b
        L = np.array([[self.l1],[self.l2],[self.l3]])
        W = [[1.0, 0, 0]]
        self.eso = ESO(A, B, W, L, q0, Tp)

    def set_b(self, b):
        self.b = b
        self.eso.set_B([[0], [b], [0]])

    def calculate_control(self, x_full, q_d, q_d_dot, q_d_ddot):
        q = x_full[self.joint_index]
        q_dot = x_full[self.joint_index + 2]

        u = getattr(self, 'last_u', 0.0)

        M = self.model.M(x_full)

        M_inv = np.linalg.inv(M)
        b_est = M_inv[self.joint_index, self.joint_index]

        self.set_b(b_est)

        self.eso.update(q, u)

        x_hat = self.eso.get_state()
        q_hat = x_hat[0]
        q_dot_hat = x_hat[1]
        f_hat = x_hat[2]

        v = q_d_ddot + self.kd * (q_d_dot - q_dot_hat) + self.kp * (q_d - q_hat)

        u = (v - f_hat) / self.b

        self.last_u = u

        return float(u)
