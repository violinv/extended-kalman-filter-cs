import numpy as np
import matplotlib.pyplot as plt

class TwoDimensionalEKF:
    """
    Implement the Extended Kalman Filter (EKF) to locate a moving object, assuming
    that we are in a 2D space with constant velocity.
    """
    
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise):
        """
        Initialise the initial state, covariance, process, and measurement noise
    
        Parameters
        -----------
        - initial_state: [x, y, vx, vy] position (x, y) and velocity (vx, vy)
            Input numpy array with initial state estimate.
        - initial_covariance: 4x4 state uncertainty matrix
        - process_noise: 4x4 uncertainty matrix in object motion
        - measurement_noise: 2x2 uncertainty vector in measurement
        """

# create class for EKF algorithm:
# - __init__ will have initial state
# - prediction step
# - update step

# plot the EKF estimates for position
# plot the EKF estimates for velocity

