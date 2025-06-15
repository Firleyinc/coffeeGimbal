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

class LowPassFilter:
    def __init__(self, alpha, initial=0.0):
        self.alpha = alpha  # Smoothing factor (0 < alpha < 1)
        self.value = initial
        
    def update(self, new_value):
        self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value

# Full control loop (no internal simulation)
class FullGimbalController:
    def __init__(self, controller_params, acceleration, jerk, dt=0.002):
        Kp, Ki, Kd = controller_params
        self.pid = PID_Controller(Kp, Ki, Kd)
        self.acceleration = acceleration  # Changed from self.a_x
        self.jerk = jerk                 # Changed from self.a_x_dot
        self.J = compute_J(m_kubek=0.1, r_kubek=0.03, m_x=0.1, l_x=0.05, m_y=0.1, l_y=0.07)
        self.dt = dt
        self.theta = 0.0
        self.theta_dot = 0.0
        self.pid.previous_error = 0.0
        self.pid.integral = 0.0
        
        # Initialize with safe values
        self.acceleration = np.clip(acceleration, -9.8, 9.8)
        self.jerk = jerk

        # Physical limits
        self.max_accel = 9.8 * 2  # 2G max
        self.max_jerk = 100.0
        self.max_tilt = np.radians(90)  # 30 degrees
        self.max_tilt_rate = np.radians(360)  # 90°/sec
        
        # State variables
        self.dt = dt
        self.theta = 0.0
        self.last_theta = 0.0
        
        # Input filtering
        self.alpha = 0.5  # Increased from 0.3 (faster response to input changes)
        self.accel_filter = LowPassFilter(alpha=0.3, initial=0.0)  # Faster acceleration filter
        self.jerk_filter = LowPassFilter(alpha=0.2, initial=0.0)   # Slightly slower jerk filter
        
        # Initialize with safe values
        self.update_inputs(acceleration, jerk)
    
    def update_inputs(self, accel, jerk):
        """Update and filter inputs with safety checks"""
        self.raw_accel = np.clip(accel, -self.max_accel, self.max_accel)
        self.raw_jerk = np.clip(jerk, -self.max_jerk, self.max_jerk)
        self.accel = self.accel_filter.update(self.raw_accel)
        self.jerk = self.jerk_filter.update(self.raw_jerk)

    def compute_feedforward(self,a_dot, J, g=9.81):
        return (J / g) * a_dot

    def compute_Plat(self, u_R, J, g=9.81):
        max_tilt = np.radians(90)  # 30 degree maximum tilt
        raw_theta = (g/J) * u_R
        return np.clip(raw_theta, -max_tilt, max_tilt)

    def sync_state(self, theta):
        self.theta = theta
        self.theta_dot = 0
        self.pid.previous_error = 0
        self.pid.integral = 0
    
    def update(self, g=9.81):
        # 1. Compute target theta from acceleration
        safe_accel = np.clip(self.accel, -0.95*g, 0.95*g)  # 5% safety margin
        target_theta = np.arcsin(safe_accel / g)
        
        # 2. Compute theta_dot from jerk
        cos_theta = max(0.1, np.cos(target_theta))  # Avoid division by zero
        target_theta_dot = self.jerk / (g * cos_theta)
        
        # 3. Feedforward control
        u_ff = 1.5 * self.compute_feedforward(target_theta_dot, self.J)
        
        # 4. PID control with anti-windup
        error = target_theta - self.theta
        u_pid = self.pid.compute(error=error, dt=self.dt)
        
        # 5. Combine and limit output
        u_total = u_ff + u_pid
        new_theta = self.compute_Plat(u_total, self.J)
        
        # 6. Rate limit the output
        max_delta = self.max_tilt_rate * self.dt
        self.theta = self.last_theta + np.clip(new_theta - self.last_theta, 
                                             -max_delta, max_delta)
        self.last_theta = self.theta
        
        return self.theta#np.clip(self.theta, -self.max_tilt, self.max_tilt)