import numpy as np
import matplotlib.pyplot as plt

# Assuming you have your FullGimbalController already imported here
from Controllers.Gimbal_Controller import FullGimbalController

# Simulated plant
class GimbalSimulator:
    def __init__(self, controller_params, acceleration, jerk, dt=0.01):
        self.controller = FullGimbalController(controller_params, acceleration, jerk, dt)
        self.dt = dt
        self.g = 9.81

    def run_simulation(self, input_profile, duration=5.0):
        time_steps = int(duration / self.dt)
        theta_log = []
        input_log = []

        for t in range(time_steps):
            t_real = t * self.dt
            accel_input, jerk_input = input_profile(t_real)
            self.controller.update_inputs(accel_input, jerk_input)
            theta = self.controller.update(self.g)
            theta_log.append(theta)
            input_log.append(accel_input)

        return np.linspace(0, duration, time_steps), np.array(theta_log), np.array(input_log)

# Sine input profile generator
def sine_input(t):
    amplitude = 3.0  # m/s²
    frequency = 0.5  # Hz
    accel = amplitude * np.sin(2 * np.pi * frequency * t)
    jerk = 2 * np.pi * frequency * amplitude * np.cos(2 * np.pi * frequency * t)
    return accel, jerk

# Sweeping function
def pid_sweep(Kp_range, Ki_range, Kd_range, plot_results=True):
    results = []

    for Kp in Kp_range:
        for Ki in Ki_range:
            for Kd in Kd_range:
                controller_params = (Kp, Ki, Kd)
                sim = GimbalSimulator(controller_params, 0.0, 0.0)
                time, theta_log, input_log = sim.run_simulation(sine_input, duration=10.0)

                overshoot = np.max(np.abs(theta_log))
                settling_time = time[np.where(np.abs(theta_log - theta_log[-1]) < 0.05)[0][0]] if np.any(np.abs(theta_log - theta_log[-1]) < 0.05) else None

                results.append({
                    'Kp': Kp,
                    'Ki': Ki,
                    'Kd': Kd,
                    'overshoot': overshoot,
                    'settling_time': settling_time
                })

                if plot_results:
                    plt.plot(time, np.degrees(theta_log), label=f'Kp={Kp}, Ki={Ki}, Kd={Kd}')
    
    if plot_results:
        plt.title("PID Sweep Results with Sine Input")
        plt.xlabel("Time (s)")
        plt.ylabel("Theta (deg)")
        plt.legend()
        plt.grid()
        plt.show()

    return results

# Run test
Kp_range = [50.0]
Ki_range = [0.5, 1.0, 1.5, 2.0, 2.5]
Kd_range = [1.5]

results = pid_sweep(Kp_range, Ki_range, Kd_range)
