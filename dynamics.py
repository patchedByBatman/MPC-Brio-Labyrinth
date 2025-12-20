import numpy as np


class Dynamics:
    NUM_STATES = 6
    NUM_INPUTS = 2

    def __init__(self, Ts = 1E-3):
        """
        All in SI units.
        states = [x, vx, y, vy, alpha, beta]
        accelerations = [ax, ay]
        inputs = [alpha_dot, beta_dot]
        """
        self.__Ts = Ts  # simulation time step in seconds
        self.__g = -9.81  # acceleration due to gravity
        self.__k1 = 1
        self.__k2 = 1

        self.__A = np.array([
            [1, self.__Ts, 0, 0, 0, 0],
            [0, 1, 0, 0, -5*self.__g*self.__Ts/ 7, 0],
            [0, 0, 1, self.__Ts, 0, 0],
            [0, 0, 0, 1, 0, -5*self.__g*self.__Ts/7],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])

        self.__B = np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [0, 0],
            [self.__k1*self.__Ts, 0],
            [0, self.__k2*self.__Ts]
        ])

        self.__states = np.zeros((self.NUM_STATES, 1))
        self.__inputs = np.zeros((self.NUM_INPUTS, 1))

    def update_inputs(self, u):
        self.__inputs[0, 0] = u[0]
        self.__inputs[1, 0] = u[1]

    def non_linear_dynamics_ct(self, states, t, w1_func, w2_func):
        # for readability
        x = states[0]
        vx = states[1]
        y = states[2]
        vy = states[3]
        a = states[4]  # alpha
        b = states[5]  # beta

        # w1_func and w2_func for time-varying inputs
        # Should take t and return input at t
        da = self.__k1 * w1_func(t)  # alpha_dot
        db = self.__k2 * w2_func(t)  # beta_dot
        
        ax = 5 * (x*da**2 + y*da*db - self.__g*np.sin(a))/7
        ay = 5 * (y*db**2 + x*da*db - self.__g*np.sin(b))/7

        return [vx, ax, vy, ay, da, db]

    def linear_dynamics_ct(self, states, t, w1_func, w2_func):
        # for readability
        x = states[0]
        vx = states[1]
        y = states[2]
        vy = states[3]
        a = states[4]  # alpha
        b = states[5]  # beta

        # w1_func and w2_func for time-varying inputs
        # Should take t and return input at t
        da = self.__k1 * w1_func(t)  # alpha_dot
        db = self.__k2 * w2_func(t)  # beta_dot
        
        ax = -5 * self.__g * a/7
        ay = -5 * self.__g * b/7

        return [vx, ax, vy, ay, da, db]
    
    def linear_dynamics_dt(self, states, t, w1_func, w2_func):
        # w1_func and w2_func for time-varying inputs
        # Should take t and return input at t
        w1 = self.__k1 * w1_func(t)  # alpha_dot
        w2 = self.__k2 * w2_func(t)  # beta_dot

        self.update_inputs([w1, w2])

        self.__states = self.__A @ self.__states + self.__B @ self.__inputs

        return self.__states

