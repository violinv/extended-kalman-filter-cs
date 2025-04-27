import numpy as np
import matplotlib.pyplot as plt

class TwoDimensionalEKF:
    """
    Implement the Extended Kalman Filter (EKF) to locate a moving object, assuming
    that we are in a 2D space with constant velocity.
    """
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise, f, h, F, H, dt=1.0):
        """
        Initialise the initial state, covariance, process, and measurement noise
    
        Parameters
        -----------
        - initial_state: [x, y, vx, vy] with position (x, y) and velocity (vx, vy)
            Input numpy array with initial state estimate.
        - initial_covariance: 4x4 state uncertainty matrix
        - process_noise: 4x4 uncertainty matrix in object motion
        - measurement_noise: 2x2 uncertainty vector in measurement
        - f: dynamic model function (non-linear)
        - h: measurement model function
        - F: Jacobian matrix of f
        - H: Jacobian matrix of h
        -dt: float representing time interval
        """

        # initialise
        self.x = initial_state
        self.P = initial_covariance
        self.Q = process_noise
        self.R = measurement_noise
        self.f = f
        self.h = h
        self.F = F
        self.H = H
        self.dt = dt

    def predict(self):
        """
        Implement the predict step of the EKF algorithm, assuming additive noise.
        """
        # apply the dynamic model function on previous state to predict the next state
        self.x = self.f(self.x)

        # predict the corresponding covariance
        F = self.F(self.x)
        self.P = F @ self.P @ F.T + self.Q

        return self.x, self.P

    def update(self, y):
        """
        Implement the predict step of the EKF algorithm.
        """
        # calculating equations as derived in mathematical report on the EKF (see README)
        v = y - self.h(self.x)
        H = self.H(self.x)
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ v
        self.P = self.P - (K @ H) @ self.P

        return self.x, self.P
        

# create class for EKF algorithm:
# - __init__ will have initial state
# - prediction step
# - update step

# plot the EKF estimates for position
# plot the EKF estimates for velocity

