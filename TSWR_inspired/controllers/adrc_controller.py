import numpy as np
from .adrc_joint_controller import ADRCJointController
from .controller import Controller

class ADRController(Controller):
    def __init__(self, Tp, params):
        self.joint_controllers = []
        for i, param in enumerate(params):
            kp, kd, p, q0 = param
            self.joint_controllers.append(
                ADRCJointController(kp, kd, p, q0, Tp, joint_index=i)
            )

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        u = []
        for i, controller in enumerate(self.joint_controllers):
            # PRZEKAZUJEMY PEŁNY STAN x, a nie tylko [q, q_dot]
            u_i = controller.calculate_control(x, q_d[i], q_d_dot[i], q_d_ddot[i])
            u.append(u_i)
        u = np.array(u)[:, np.newaxis]
        return u