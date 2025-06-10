import matplotlib.pyplot as plt
import numpy as np
from numpy import pi
from scipy.integrate import odeint

from controllers.adrc_controller import ADRController

from trajectory_generators.constant_torque import ConstantTorque
from trajectory_generators.sinusonidal import Sinusoidal
from trajectory_generators.poly3 import Poly3
from utils.simulation import simulate

Tp = 0.001
end = 5

#traj_gen = ConstantTorque(np.array([0., 1.0])[:, np.newaxis])
#traj_gen = Sinusoidal(np.array([0., 1.]), np.array([2., 2.]), np.array([0., 0.]))
traj_gen = Poly3(np.array([0., 0.]), np.array([pi/4, pi/6]), end)

b_est_1 = None
b_est_2 = None

kp_est_1 = 0.5
kp_est_2 = 1.0

kd_est_1 = 0.4
kd_est_2 = 0.9

p1 = 70
p2 = 115

q0, qdot0, _ = traj_gen.generate(0.)
q1_0 = np.array([q0[0], qdot0[0]])
q2_0 = np.array([q0[1], qdot0[1]])
controller = ADRController(Tp, params=[[ kp_est_1, kd_est_1, p1, q1_0],
                                       [ kp_est_2, kd_est_2, p2, q2_0]]) #b_est 1 i 2

Q, Q_d, u, T = simulate("PYBULLET", traj_gen, controller, Tp, end)

eso1 = np.array(controller.joint_controllers[0].eso.states)
eso2 = np.array(controller.joint_controllers[1].eso.states)

# Plot the states (ESO vs. Actual joint positions)
plt.subplot(221)
plt.plot(T, eso1[:, 0])  # ESO estimated position for joint 1
plt.plot(T, Q[:, 0], 'r')  # Actual position for joint 1 (desired)
plt.title('Joint 1: ESO Estimated Position vs Actual Position')

plt.subplot(222)
plt.plot(T, eso1[:, 1])  # ESO estimated velocity for joint 1
plt.plot(T, Q[:, 2], 'r')  # Actual velocity for joint 1 (desired)
plt.title('Joint 1: ESO Estimated Velocity vs Actual Velocity')

plt.subplot(223)
plt.plot(T, eso2[:, 0])  # ESO estimated position for joint 2
plt.plot(T, Q[:, 1], 'r')  # Actual position for joint 2 (desired)
plt.title('Joint 2: ESO Estimated Position vs Actual Position')

plt.subplot(224)
plt.plot(T, eso2[:, 1])  # ESO estimated velocity for joint 2
plt.plot(T, Q[:, 3], 'r')  # Actual velocity for joint 2 (desired)
plt.title('Joint 2: ESO Estimated Velocity vs Actual Velocity')

plt.tight_layout()  # Adjust layout for better spacing
plt.show()

# Plot joint trajectories and control input signals
plt.subplot(221)
plt.plot(T, Q[:, 0], 'r')  # Actual position for joint 1
plt.plot(T, Q_d[:, 0], 'b')  # Desired position for joint 1
plt.title('Joint 1: Actual Position vs Desired Position')

plt.subplot(222)
plt.plot(T, Q[:, 1], 'r')  # Actual position for joint 2
plt.plot(T, Q_d[:, 1], 'b')  # Desired position for joint 2
plt.title('Joint 2: Actual Position vs Desired Position')

plt.subplot(223)
plt.plot(T, u[:, 0], 'r')  # Control input for joint 1
plt.plot(T, u[:, 1], 'b')  # Control input for joint 2
plt.title('Control Inputs for Joint 1 and Joint 2')

plt.tight_layout()  # Adjust layout for better spacing
plt.show()
