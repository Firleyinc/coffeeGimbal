import matplotlib.pyplot as plt
import numpy as np
from controllers.adrc_flc_controller import ADRFLController
from trajectory_generators.constant_torque import ConstantTorque
from trajectory_generators.sinusonidal import Sinusoidal
from trajectory_generators.poly3 import Poly3
from utils.simulation import simulate

Tp = 0.001
end = 5

# traj_gen = ConstantTorque(np.array([0., 1.0])[:, np.newaxis])
traj_gen = Sinusoidal(np.array([0., 1.]), np.array([2., 2.]), np.array([0., 0.]))
#traj_gen = Poly3(np.array([0., 0.]), np.array([np.pi/4, np.pi/6]), end)

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

Kp = np.diag([kp_est_1, kp_est_2])
Kd = np.diag([kd_est_1, kd_est_2])
p = np.array([p1, p2])

controller = ADRFLController(Tp, np.concatenate([q0, qdot0]), Kp, Kd, p)


Q, Q_d, u, T = simulate("PYBULLET", traj_gen, controller, Tp, end)

eso = np.array(controller.eso.states)

# First figure: ESO states vs actual joint states Q
plt.figure(figsize=(12, 8))

plt.subplot(221)
plt.plot(T, eso[:, 0], label='ESO state 0')
plt.plot(T, Q[:, 0], 'r', label='Q joint 0')
plt.title('Joint 0: ESO State vs Actual')
plt.xlabel('Time [s]')
plt.ylabel('Position')
plt.legend()
plt.grid(True)

plt.subplot(222)
plt.plot(T, eso[:, 2], label='ESO state 2')
plt.plot(T, Q[:, 2], 'r', label='Q dot joint 0')
plt.title('Joint 0 Velocity: ESO vs Actual')
plt.xlabel('Time [s]')
plt.ylabel('Velocity')
plt.legend()
plt.grid(True)

plt.subplot(223)
plt.plot(T, eso[:, 1], label='ESO state 1')
plt.plot(T, Q[:, 1], 'r', label='Q joint 1')
plt.title('Joint 1: ESO State vs Actual')
plt.xlabel('Time [s]')
plt.ylabel('Position')
plt.legend()
plt.grid(True)

plt.subplot(224)
plt.plot(T, eso[:, 3], label='ESO state 3')
plt.plot(T, Q[:, 3], 'r', label='Q dot joint 1')
plt.title('Joint 1 Velocity: ESO vs Actual')
plt.xlabel('Time [s]')
plt.ylabel('Velocity')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# Second figure: Desired vs Actual trajectories and control inputs
plt.figure(figsize=(12, 8))

plt.subplot(221)
plt.plot(T, Q[:, 0], 'r', label='Actual joint 0')
plt.plot(T, Q_d[:, 0], 'b', label='Desired joint 0')
plt.title('Joint 0 Position: Actual vs Desired')
plt.xlabel('Time [s]')
plt.ylabel('Position')
plt.legend()
plt.grid(True)

plt.subplot(222)
plt.plot(T, Q[:, 1], 'r', label='Actual joint 1')
plt.plot(T, Q_d[:, 1], 'b', label='Desired joint 1')
plt.title('Joint 1 Position: Actual vs Desired')
plt.xlabel('Time [s]')
plt.ylabel('Position')
plt.legend()
plt.grid(True)

plt.subplot(223)
plt.plot(T, u[:, 0], 'r', label='Control input joint 0')
plt.plot(T, u[:, 1], 'b', label='Control input joint 1')
plt.title('Control Inputs')
plt.xlabel('Time [s]')
plt.ylabel('Torque')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()