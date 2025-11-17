"""
Single-link inverted pendulum simulation with PD and LQR control.

Author: Jaehyun Park
GitHub: https://github.com/blueed9696/Pendulum_Control
License: MIT
"""
import numpy as np

class Model():
    def __init__(self):
        self.rod_length = 1.0           # rod length
        self.ball_radius = 0.05 
        self.ball_mass = 1.0 
        self.rod_thickness = 0.01
        
        self.gravity = 9.81

    def state_space_model(self):
        g, L, m = self.gravity, self.rod_length, self.ball_mass
        A = np.array([[0, 1],
                      [g/L, 0]])
        B = np.array([[0],
                      [1/(m*L*L)]])
        
        return A, B
