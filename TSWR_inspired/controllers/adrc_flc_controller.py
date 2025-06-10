import numpy as np
from observers.eso import ESO
from .controller import Controller
from model.model_definition import GimbalModel

class ADRFLController(Controller):
    def __init__(self, Tp, q0, Kp, Kd, p):
        self.model = GimbalModel(Tp)
        self.Kp = Kp
        self.Kd = Kd
        self.Tp = Tp
        self.p = p

        q, q_dot = q0[:2], q0[2:]
        self.eso = None
        self.update_params(q, q_dot)

    def update_params(self, q, q_dot):
        x = np.hstack([q, q_dot])
        M_hat = self.model.M(x)
        C_hat = self.model.C(x)

        I2 = np.eye(2)
        Z2 = np.zeros((2, 2))

        A = np.block([
            [Z2, I2, Z2],
            [Z2, -np.linalg.inv(M_hat) @ C_hat, I2],
            [Z2, Z2, Z2]
        ])

        B = np.vstack([
            Z2,
            np.linalg.inv(M_hat),
            Z2
        ])

        W = np.hstack([I2, np.zeros((2, 4))])

        L = np.eye(6, 2) * 10.0

        initial_state = np.hstack([q, q_dot, np.zeros(2)])

        if self.eso is None:
            self.eso = ESO(A, B, W, L, initial_state, self.Tp)
        else:
            self.eso.A = A
            self.eso.B = B
            self.eso.W = W
            self.eso.L = L

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        q = x[:2]
        q_dot = x[2:]
        self.update_params(q, q_dot)

        z_hat = self.eso.get_state()
        q_hat = z_hat[:2]
        q_d_hat = z_hat[2:4]
        f_hat = z_hat[4:]

        e = q_d - q
        e_dot = q_d_dot - q_dot

        v = self.Kp @ e + self.Kd @ e_dot + q_d_ddot

        M_hat = self.model.M(np.hstack([q, q_dot]))
        C_hat = self.model.C(np.hstack([q, q_dot]))

        u = M_hat @ (v - f_hat) + C_hat @ q_d_hat
        self.eso.update(q, u)
        return u
