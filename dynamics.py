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

    def set_initial_states(self, states0):
        x, vx, y, vy, a, b = states0

        self.__states[0, 0] = x
        self.__states[1, 0] = vx
        self.__states[2, 0] = y
        self.__states[3, 0] = vy
        self.__states[4, 0] = a
        self.__states[5, 0] = b

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
    
    def linear_dynamics_dt(self, t, w1_func, w2_func):
        # w1_func and w2_func for time-varying inputs
        # Should take t and return input at t
        w1 = w1_func(t)  # alpha_dot
        w2 = w2_func(t)  # beta_dot

        self.update_inputs([w1, w2])

        self.__states = self.__A @ self.__states + self.__B @ self.__inputs

        return self.__states


if __name__ == "__main__":
    import numpy as np
    from scipy.integrate import odeint
    from matplotlib import pyplot as plt

    from dynamics import Dynamics


    f = 0.001
    gain = 1
    tsim = np.linspace(0, 10, int(10E3))
    w1 = - 2 * np.pi * f * gain * np.sin(2 * np.pi * f * tsim)
    w2 = 2 * np.pi * f * gain * np.cos(2 * np.pi * f * tsim)

    w1_func = lambda t: np.interp(t, tsim, w1)
    w2_func = lambda t: np.interp(t, tsim, w2)

    dyn = Dynamics()
    y0 = [0, 0, 0, 0, gain, 0]

    states, info = odeint(func=dyn.non_linear_dynamics_ct, y0=y0, t=tsim, full_output=True, args=(w1_func, w2_func))
    plt.plot(states[:, 0], states[:, 2])
    states, info = odeint(func=dyn.linear_dynamics_ct, y0=y0, t=tsim, full_output=True, args=(w1_func, w2_func))
    plt.plot(states[:, 0], states[:, 2])
    states = np.zeros((dyn.NUM_STATES, tsim.shape[0]))
    dyn.set_initial_states(y0)
    for i, t in enumerate(tsim):
        states[:, i] = dyn.linear_dynamics_dt(t, w1_func, w2_func).reshape((6, ))
    plt.plot(states[0, :], states[2, :])
    plt.show()