import numpy as np

class GimbalDynamics:
    def __init__(self):
        # Moments of inertia (pitch gimbal)
        self.Jax = 1.0  # x-axis inertia
        self.Jay = 1.0  # y-axis inertia (pitch axis)
        self.Jaz = 1.0  # z-axis inertia
        
        # Products of inertia (pitch gimbal)
        self.Dxy = 0.0
        self.Dxz = 0.0
        self.Dyz = 0.0
        
        # Yaw gimbal inertia parameters
        self.Jkx = 1.0
        self.Jky = 1.0
        self.Jkz = 1.0
        self.dxy = 0.0
        self.dxz = 0.0
        self.dyz = 0.0
        
        # State: [θ_pitch, θ_yaw, ω_pitch, ω_yaw, ω_roll_body, ω_pitch_body, ω_yaw_body]
        self.state = np.zeros(7)

    def M(self, state):
        """
        Mass matrix incorporating both pitch and yaw dynamics with cross-coupling
        """
        θ2 = state[1]  # yaw gimbal angle
        
        # Trigonometric terms
        sin_θ2 = np.sin(θ2)
        cos_θ2 = np.cos(θ2)
        sin_2θ2 = np.sin(2*θ2)
        cos_2θ2 = np.cos(2*θ2)
        
        # Pitch channel inertia (Jay)
        M11 = self.Jay
        
        # Yaw channel inertia (Jk from equation 15)
        M22 = (self.Jkz + self.Jax*sin_θ2**2 + self.Jaz*cos_θ2**2 - 
            self.Dxz*sin_2θ2)
        
        # Cross-coupling terms
        M12 = self.Dyz*sin_θ2 + self.Dxy*cos_θ2
        M21 = M12  # Symmetric
        
        return np.array([[M11, M12],
                        [M21, M22]])

    def C(self, state):
        """
        Coriolis and centrifugal matrix including all coupling effects
        """
        θ2 = state[1]
        ω_pitch = state[2]
        ω_yaw = state[3]
        ω_roll_body = state[4]
        ω_pitch_body = state[5]
        ω_yaw_body = state[6]
        
        # Transform body rates to yaw gimbal frame (equation 36)
        pk = ω_roll_body*np.cos(θ2) + ω_pitch_body*np.sin(θ2)
        qk = -ω_roll_body*np.sin(θ2) + ω_pitch_body*np.cos(θ2)
        rk = ω_yaw_body + state[3]  # yaw gimbal rate
        
        # Trigonometric terms
        sin_θ2 = np.sin(θ2)
        cos_θ2 = np.cos(θ2)
        sin_2θ2 = np.sin(2*θ2)
        cos_2θ2 = np.cos(2*θ2)
        
        # Pitch channel terms (from equation 6)
        C11 = 0
        C12 = ((self.Jaz - self.Jax)*pk*rk + 
            self.Dxz*(pk**2 - rk**2) - 
            self.Dyz*(ω_yaw_body - pk*ω_pitch) - 
            self.Dxy*(ω_roll_body + ω_pitch*rk))
        
        # Yaw channel terms (Td1 + Td2 components)
        C21 = -((self.Jkx + self.Jax*cos_θ2**2 + self.Jaz*sin_θ2**2 + 
                self.Dxz*sin_2θ2 - (self.Jky + self.Jay))*pk*qk)
        
        C22 = ((self.dxz + (self.Jaz - self.Jax)*sin_θ2*cos_θ2 + 
                self.Dxz*cos_2θ2)*(ω_roll_body - qk*rk) + 
            (self.dyz + self.Dyz*cos_θ2 - self.Dxy*sin_θ2)*(ω_pitch_body + pk*rk) + 
            (self.dxy + self.Dxy*cos_θ2 + self.Dyz*sin_θ2)*(pk**2 - qk**2))
        
        return np.array([[C11, C12],
                        [C21, C22]])
    
    def predict(self, x):
        """
        Predicts the next state x_mi given the current state x using the model dynamics.
        """
        M = self.M(x)  # Mass matrix
        C = self.C(x)  # Coriolis matrix
        
        q_dot = x[2:]  # Extract velocity (q1_dot, q2_dot)
        
        # Compute acceleration (assuming no external forces for prediction)
        q_ddot = np.linalg.inv(M) @ (-C @ q_dot[:, np.newaxis])
        
        # Predict next state using Euler integration
        x_pred = x + np.hstack((q_dot, q_ddot.flatten())) * self.Tp
        
        return x_pred