# Based on the husky robot:
# https://github.com/yz9/wall_following_pid_robot


class PID:
    """
    A PID controller.

    Intutive explanation: https://www.youtube.com/watch?v=wkfEZmsQqiA.
    """

    def __init__(self, Kp, Ti, Td, dt):
        self.Kp = Kp
        self.Td = Td
        self.Ti = Ti
        self.curr_error = 0
        self.prev_error = 0
        self.sum_error = 0
        self.prev_error_deriv = 0
        self.curr_error_deriv = 0
        self.control = 0
        self.dt = dt

    def update_control(self, current_error, reset_prev=False):
        if(self.Td != 3.0):
            print("TD is changed", self.Td)
        self.curr_error_deriv = (self.curr_error - self.prev_error) / self.dt

        # steering angle = P gain + D gain + I gain
        p_gain = self.Kp * self.curr_error
        i_gain = self.sum_error + self.Ti * self.curr_error * self.dt
        self.sum_error = i_gain
        d_gain = self.Td * self.curr_error_deriv

        # PID control
        w = p_gain + d_gain + i_gain
        self.control = w

        # update error
        self.prev_error = self.curr_error
        self.curr_error = current_error
        self.prev_error_deriv = self.curr_error_deriv
        return self.control
