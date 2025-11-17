"""
Single-link inverted pendulum simulation with PD and LQR control.

Author: Jaehyun Park
GitHub: https://github.com/blueed9696/Pendulum_Control
License: MIT
"""
import numpy as np
from scipy.linalg import solve_continuous_are, expm
from model import Model
import cvxpy as cp

class Controller():
    def __init__(self, dt):
        ############# PD Control #############
        self.Kp = 20
        self.Kd = 5
        self.dt = dt
        ################################
        
        ############# LQR #############
        self.Q = np.diag([100, 1])
        self.R = np.array([[0.01]])

        model = Model()
        self.A, self.B = model.state_space_model()

        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.inv(self.R) @ self.B.T @ P
        ################################

        ############# MPC #############
        self.N = 5        # horizon length
        self.umax = 50     # input boundary (|u| <= umax)
        
        self.Q_mpc  = np.diag([100.0, 1.0])
        self.R_mpc  = np.array([[0.01]])
        self.Qf_mpc = self.Q_mpc.copy()

        self.dt_mpc = 1.0 / 60.0 

        self.Ad, self.Bd = self.discretize(self.dt_mpc)
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

    def discretize(self, dt):
        n = self.A.shape[0]
        m = self.B.shape[1]

        # Bolock matrix exponential form : e^MT = [[A B];[0 I]]
        M = np.block([
            [self.A, self.B],
            [np.zeros((m, n + m))]
        ])

        Md = expm(M * dt)

        Ad = Md[:n, :n]
        Bd = Md[:n, n:n+m]
        return Ad, Bd
    
    def linear_MPC(self, theta, theta_d):
        x0 = np.array([theta, theta_d])     # Current state

        n = self.Ad.shape[0]  # state dim = 2
        N = self.N

        # Optimization variables
        x = cp.Variable((n, N + 1))
        u = cp.Variable((1, N))

        cost = 0
        constraints = []

        # Initial condition
        constraints += [x[:, 0] == x0]

        # stage costs + dynamics + input bounds
        for k in range(N):
            cost += cp.quad_form(x[:, k], self.Q_mpc) + cp.quad_form(u[:, k], self.R_mpc)
            constraints += [x[:, k+1] == self.Ad @ x[:, k] + self.Bd @ u[:, k]]
            constraints += [cp.abs(u[:, k]) <= self.umax]

        # terminal cost
        cost += cp.quad_form(x[:, N], self.Qf_mpc)

        # Solve QP
        prob = cp.Problem(cp.Minimize(cost), constraints)
        prob.solve(solver=cp.OSQP, warm_start=True)

        u_opt = u.value  # shape (1, N)

        return float(u_opt[0, 0])
