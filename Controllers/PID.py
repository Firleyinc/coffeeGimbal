class PID_Controller:
    def __init__(self, Kp, Ki, Kd, y):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.previous_error = 0
        self.integral = 0

    def compute(self, error, dt=0.1):
        
        # Proportional term
        P_out = self.Kp * error
        
        # Integral term
        self.integral += error * dt
        I_out = self.Ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        D_out = self.Kd * derivative
        
        # Compute total output
        u = P_out + I_out + D_out
        
        # Update previous error
        self.previous_error = error
        
        return u