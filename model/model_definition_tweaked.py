import numpy as np


class GimbalModel:
    def __init__(self, Tp):
        self.Tp = Tp  # Sampling time

        # === Placeholder physical parameters ===
        # Yaw gimbal
        self.l1 = None  # Length of yaw gimbal (if applicable)
        self.m1 = None  # Mass of yaw gimbal
        self.r1 = None  # Radius of yaw gimbal
        self.Jkz = None  # Inertia around z (yaw) axis

        # Pitch gimbal
        self.l2 = None  # Length of pitch gimbal
        self.m2 = None  # Mass of pitch gimbal
        self.r2 = None  # Radius of pitch gimbal
        self.Jax = None  # Inertia around x-axis (optical axis)
        self.Jay = None  # Inertia around y-axis (lateral axis)
        self.Jaz = None  # Inertia around z-axis

        # === Example inertia equations (commented) ===
        # For a solid cylinder rotating about central axis:
        # self.Jkz = 0.5 * self.m1 * self.r1 ** 2
        
        # For a solid rod rotating about center perpendicular to length:
        # self.Jax = (1 / 12) * self.m2 * self.l2 ** 2  # if x-axis is through length
        # self.Jay = (1 / 12) * self.m2 * self.l2 ** 2
        # self.Jaz = (1 / 2) * self.m2 * self.r2 ** 2  # if rotating about cylindrical z-axis

    def M(self, x):
        """
        Inertia (mass) matrix M(q) for 2-DOF yaw-pitch gimbal.
        x = [θ_yaw, θ_pitch, ω_yaw, ω_pitch]
        """
        θ1, θ2, _, _ = x

        # Placeholder formulas
        sin_θ2 = np.sin(θ2)
        cos_θ2 = np.cos(θ2)

        M11 = self.Jkz + self.Jax * sin_θ2**2 + self.Jay * cos_θ2**2
        M12 = (self.Jay - self.Jax) * sin_θ2 * cos_θ2
        M21 = M12
        M22 = self.Jaz

        M = np.array([
            [M11, M12],
            [M21, M22]
        ])
        return M

    def C(self, x):
        """
        Coriolis and centrifugal matrix C(q, q̇).
        x = [θ_yaw, θ_pitch, ω_yaw, ω_pitch]
        """
        θ1, θ2, ω1, ω2 = x

        sin_θ2 = np.sin(θ2)
        cos_θ2 = np.cos(θ2)

        d = (self.Jay - self.Jax) * sin_θ2 * cos_θ2

        C11 = 0
        C12 = -d * ω2
        C21 = d * ω1
        C22 = 0

        C = np.array([
            [C11, C12],
            [C21, C22]
        ])
        return C

    def predict(self, x, tau=np.zeros(2)):
        """
        Predict next state using Euler integration of dynamics.
        x = [θ_yaw, θ_pitch, ω_yaw, ω_pitch]
        tau = [τ_yaw, τ_pitch] external torques (optional)
        """
        q_dot = x[2:]  # [ω_yaw, ω_pitch]

        M = self.M(x)  # Inertia matrix
        C = self.C(x)  # Coriolis matrix

        # Compute angular accelerations: q̈ = M⁻¹(τ - C·q̇)
        q_ddot = np.linalg.inv(M) @ (tau - C @ q_dot[:, np.newaxis])
        q_ddot = q_ddot.flatten()

        # Euler integration
        x_pred = x + np.hstack((q_dot, q_ddot)) * self.Tp
        return x_pred
