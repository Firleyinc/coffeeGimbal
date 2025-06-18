import numpy as np
from Controllers.PID import PID_Controller
from Controllers.utils.calculate_inertia import compute_J

class LowPassFilter:
    def __init__(self, alpha, initial=0.0):
        self.alpha = alpha
        self.value = initial
        
    def update(self, new_value):
        self.value = self.alpha * new_value + (1 - self.alpha) * self.value
        return self.value

class DerivativeFilter:
    def __init__(self, alpha=0.1, initial=0.0):
        self.alpha = alpha
        self.value = initial
        self.last_input = initial

    def update(self, new_input, dt):
        raw_derivative = (new_input - self.last_input) / dt
        self.value = self.alpha * raw_derivative + (1 - self.alpha) * self.value
        self.last_input = new_input
        return self.value

class FullGimbalController:
    def __init__(self, controller_params, acceleration, jerk, dt=0.01):
        Kp, Ki, Kd = controller_params
        self.pid = PID_Controller(Kp, Ki, Kd)
        self.dt = dt

        self.J = 0.001485

        self.max_accel = 9.8 * 2  
        self.max_jerk = 200.0
        self.max_tilt = np.radians(90)  
        self.max_tilt_rate = np.radians(200)  

        self.accel_filter = LowPassFilter(alpha=0.6, initial=acceleration)
        self.jerk_filter = LowPassFilter(alpha=0.3, initial=jerk)
        self.velocity_filter = LowPassFilter(alpha=0.2, initial=0.0)

        self.theta = 0.0
        self.last_theta = 0.0

        self.velocity_estimate = 0.0
        self.K_velocity = 4.0 # Zacznij np. od 1.0-2.0, potem zwiększaj/dobieraj

        self.update_inputs(acceleration, jerk)

    def update_inputs(self, accel, jerk):
        self.raw_accel = np.clip(accel, -self.max_accel, self.max_accel)
        self.raw_jerk = np.clip(jerk, -self.max_jerk, self.max_jerk)
        self.accel = self.accel_filter.update(self.raw_accel)
        self.jerk = self.jerk_filter.update(self.raw_jerk)

    
    def compute_Plat(self, u_R, g=9.81):
        return (g / self.J) * u_R

    def compute_feedforward(self, jerk, g=9.81):
        # Klasyczna kompensacja przyspieszenia przez nachylenie
        return np.arcsin(np.clip(jerk / g, -0.95, 0.95))


    def sync_state(self, theta):
        self.theta = theta
        self.last_theta = theta
        self.pid.previous_error = 0
        self.pid.integral = 0
        self.velocity_estimate = 0.0

    def update(self, g=9.81):
        # Całkuj przyspieszenie do estymaty prędkości, filtruj!
        self.velocity_estimate += self.accel * self.dt
        self.velocity_estimate = self.velocity_filter.update(self.velocity_estimate)

        # Feedforward na bazie przyspieszenia
        theta_ff = self.compute_feedforward(self.jerk, g)
        # Feedforward na bazie prędkości (anticipatory term)

        velocity_term = -self.K_velocity * self.velocity_estimate

        # Sumaryczny sygnał referencyjny (theta, którą powinna mieć tacka)

        theta_ref = np.arcsin(np.clip(self.accel / g, -0.95, 0.95)) + velocity_term

        # Sterowanie PID (możesz wyłączyć P/I/D na testy, żeby zobaczyć tylko feedforward)
        error = theta_ref - self.last_theta  # Our goal is to drive this to 0

        u_pid = self.pid.compute(error, self.dt)

        u_ref = theta_ff + u_pid

        # Aktualizacja nachylenia
        self.theta += self.compute_Plat(u_ref)

        # Limit prędkości zmiany kąta (ochrona)
        max_delta = self.max_tilt_rate * self.dt
        self.theta = self.last_theta + np.clip(self.theta - self.last_theta, -max_delta, max_delta)
        self.last_theta = self.theta

        # ---- LOGGING ----
        print(f"[DBG] a_x={self.accel:.3f}, v_est={self.velocity_estimate:.3f}, "
              f"theta_ff_acc={theta_ff:.3f}, v_term={velocity_term:.3f}, "
              f"theta_out={self.theta:.3f}")

        return self.theta
