class PID_Controller:
    def __init__(self, Kp, Ki, Kd, integral_decay=0.7):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.previous_error = 0
        self.integral = 0
        self.integral_decay = integral_decay  # Factor ∈ [0, 1]

    def compute(self, error, dt=0.1):
        # Proportional term
        P_out = self.Kp * error

        # Leaky Integral term
        self.integral = self.integral_decay * self.integral + error * dt
        I_out = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.previous_error) / dt
        D_out = self.Kd * derivative

        # Compute total output
        u = P_out + I_out + D_out

        # Update previous error
        self.previous_error = error

        return u
