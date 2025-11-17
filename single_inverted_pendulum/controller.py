import numpy as np
from scipy.linalg import solve_continuous_are
from model import Model

class Controller():
    def __init__(self, dt):
        ############# PD Control #############
        self.Kp = 10
        self.Kd = 0.5
        self.dt = dt
        ################################
        
        ############# LQR #############
        self.Q = np.diag([100, 1])
        self.R = np.array([[0.01]])

        model = Model()
        A, B = model.state_space_model()

        P = solve_continuous_are(A, B, self.Q, self.R)
        self.K = np.linalg.inv(self.R) @ B.T @ P
        ################################


    def PD_control(self, theta, theta_dot, theta_des = 0, theta_dot_des = 0):
        error = theta - theta_des
        error_dot = theta_dot - theta_dot_des
        tau = self.Kp * error + self.Kd * error_dot

        return tau

    def LQR(self, theta, theta_d):
        # state error relative to upright (theta = 0)
        x = np.array([theta, theta_d])   
        tau = - self.K @ x               
        return float(tau)    

