import numpy as np
from Controllers.PID import PID_Controller
from Controllers.utils.calculate_inertia import compute_J

'''# Gimbal Plant: outputs acceleration
class GimbalPlant:
    def __init__(self, g, J):
        self.g = g
        self.J = J
        self.Y = 0

# Feedforward controller
def compute_feedforward(a_dot, J, g=9.81):
    return (J / g) * a_dot

def compute_Plat(u_R, J, g=9.81):
    return (g/J) * u_R'''

# Full control loop (no internal simulation)
class FullGimbalController:
    def __init__(self, controller_params, acceleration, jerk , dt=0.002):
        Kp, Ki, Kd = controller_params
        self.pid = PID_Controller(Kp, Ki, Kd)
        self.a = acceleration
        self.a_dot = jerk
        self.J = compute_J(m_kubek=0.1, r_kubek=0.03, m_x=0.1, l_x=0.05, m_y=0.1, l_y=0.07)
        self.dt = dt
        self.Y = 0

    def compute_feedforward(self,a_dot, J, g=9.81):
        return (J / g) * a_dot

    def compute_Plat(self,u_R, J, g=9.81):
        return (g/J) * u_R

    '''def trig_transform(self, a_ball):
        # Map acceleration to target tilt angle
        return np.arcsin(np.clip(a_ball / 9.81, -1.0, 1.0))'''
    
    def update(self,g=9.81):
        theta = np.arcsin(np.clip(self.a / 9.81, -1.0, 1.0))
        theta_dot = self.a_dot / 9.81#(9.81 * np.cos(theta))

        #===Feed Forward===
        u_FF = self.compute_feedforward(theta_dot,self.J)

        #===Main Branch===
        error = theta - self.Y

        u_PID = self.pid.compute(error=error, dt=self.dt)

        u_R = u_FF + u_PID

        self.Y = self.compute_Plat(u_R,self.J)

        return self.Y


        
