class Controller():
    def __init__(self, dt):
        self.Kp = 10
        self.Kd = 0.1
        self.prev_error = 0
        self.dt = dt

    def PD_control(self, theta, theta_des):
        error = theta - theta_des
        error_dt = (error - self.prev_error)/self.dt
        self.prev_error = error

        tau = self.Kp * error + self.Kd * error_dt

        return tau

    def LQR(self):
        pass