import numpy as np
import math


class KalmanFilter:
    def __init__(self,
                 sampling_frequency=10,
                 u0=np.zeros([2, 1]),
                 x_tilde_0=np.zeros((5, 1)),
                 P_0=np.eye(5)*100,
                 cache_values=False,
                 overwrite_x_MU=None,
                 overwrite_sigma_MU=None):
        self.__fs = sampling_frequency
        self.__Ts = 1/self.__fs
        self.__is_yt_not_nan = True

        # Normalized Throttle reference, range [0, 1]
        self.__ut = u0
        # System dynamics
        self.__At = self.__update_At()
        self.__C = np.array([
            [1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0]
            ])

        # Process noise
        self.__Q = np.diagflat([1e-3, 1, 1e-3, 1, 10])
        # Measurement noise
        self.__R = np.diagflat([1e-2, 1e-2])

        # initial conditions
        self.__x_hat_0_minus1 = x_tilde_0
        self.__sigma_0_minus1 = P_0

        # Measurement update
        self.__x_MU = overwrite_x_MU
        self.__sigma_MU = overwrite_sigma_MU

        # Time update
        self.__x_TU = x_tilde_0
        self.__sigma_TU = P_0


        self.__cache_values = cache_values
        
        # Measurement update cache
        self.__x_MU_cache = []
        self.__sigma_MU_cache = []

        # Time update cache
        self.__x_TU_cache = []
        self.__sigma_TU_cache = []

        # cache first TU from above
        self.__cache_TU_values()

    def __cache_MU_values(self):
        if self.__cache_values:
            self.__x_MU_cache.append(self.__x_MU)
            self.__sigma_MU_cache.append(self.__sigma_MU)

    def __cache_TU_values(self):
        if self.__cache_values:                
            self.__x_TU_cache.append(self.__x_TU)
            self.__sigma_TU_cache.append(self.__sigma_TU)

    def __update_At(self):
        """Update state matrix At. 
        NOTE: This function should be called whenever ut is updated.
        """
        alpha = self.__ut[0, 0]
        beta = self.__ut[1, 0]
        self.__At = np.array([
            [1, self.__Ts, 0, 0, 0],
            [0, 1, 0, 0, -5*self.__Ts*np.sin(alpha)/ 7],
            [0, 0, 1, self.__Ts, 0],
            [0, 0, 0, 1, -5*self.__Ts*np.sin(beta)/ 7],
            [0, 0, 0, 0, 1]
        ])

    def __update_ut(self, ut):
        """Update the system input.

        :param ut: Current inputs.
        """
        self.__ut = ut

    def measurement_update_cache(self):
        """returns Measurement update cache data.

        :return: list of measurement update cache data.
        """
        return self.__x_MU_cache, self.__sigma_MU_cache

    def time_update_cache(self):
        """returns Time update cache data.

        :return: list of time update cache data.
        """
        return self.__x_TU_cache, self.__sigma_TU_cache

    def __measurement_update(self,
                             y_t):
        """Does measurement update step of the Kalman filter.

        :param y_t: Current position measurement.
        """
        # this is just a number
        the_inv = self.__C@self.__sigma_TU@self.__C.T + self.__R
        the_inv = np.linalg.inv(the_inv)
        temp = self.__sigma_TU@self.__C.T@the_inv
        self.__x_MU = self.__x_TU + temp@(y_t - self.__C@self.__x_TU)
        self.__sigma_MU = self.__sigma_TU - temp@self.__C@self.__sigma_TU
        self.__cache_MU_values()

    def __time_update(self):
        """Does time update step of the kalman filter.
        """
        self.__x_TU = self.__At@self.__x_MU
        self.__sigma_TU = self.__At@self.__sigma_MU@self.__At.T + self.__Q
        self.__cache_TU_values()

    def reset(self):
        """reset the kalman filter velocity estimate.
        NOTE: only call this if the drone is close to ground and/ or the system dynamics are altered.
        """
        self.__x_MU[1, 0] = 0.

    def update(self, ut, y_t):
        """Runs the Kalman filter for one step

        :param Tt: Current normalized throttle reference.
        :param pitch_rad: Current drone pitch in radians.
        :param roll_rad: Current drone roll in radians.
        :param y_t: Current altitude measurement in meters.
        :return: State estimate.
        """
        # Check if altitude measurement is nan or outlier (which is indicated by -1/1000).
        self.__is_yt_not_nan = not (np.isnan(y_t).any())
        # Update the system input.
        self.__update_ut(ut)
        # update the system matrix At.
        self.__update_At()
        # only do the measurement update if a valid measurement is received.
        if self.__is_yt_not_nan:
            # Do the measurement update.
            self.__measurement_update(y_t)
        self.__time_update()  # Do the time update.
        # return measurement update estimate if a valid measurement is received else return time update estimate.
        return self.__x_MU if self.__is_yt_not_nan else self.__x_TU
    

if __name__ == "__main__":
    import numpy as np
    from scipy.integrate import odeint
    from matplotlib import pyplot as plt
    _, sub_plots = plt.subplots(1, 2)

    import sys, os
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

    from dynamics import Dynamics
    from kf import KalmanFilter

    f = 0.001
    gain = 1
    fs = 1000
    sim_time = 5

    tsim = np.linspace(0, sim_time, sim_time*fs)
    w1 = - 2 * np.pi * f * gain * np.sin(2 * np.pi * f * tsim)
    w2 = 2 * np.pi * f * gain * np.cos(2 * np.pi * f * tsim)

    w1_func = lambda t: np.interp(t, tsim, w1)
    w2_func = lambda t: np.interp(t, tsim, w2)

    dyn = Dynamics()
    y0 = [0, 0, 0, 0, gain, 0]

    states, info = odeint(func=dyn.non_linear_dynamics_ct, y0=y0, t=tsim, full_output=True, args=(w1_func, w2_func))

    noise = np.random.randn
    measurement_noise = 1e-2*np.random.multivariate_normal(mean=np.zeros(6), cov=5e-3*np.eye(6), size=len(states))
    measured_states = states + measurement_noise
    kf = KalmanFilter(sampling_frequency=fs,
                    u0=np.zeros([2, 1]),
                    x_tilde_0=np.zeros([5, 1]),
                    cache_values=True)

    for idx, t in enumerate(tsim):
        kf.update(np.array([[measured_states[idx, 4]], [measured_states[idx, 5]]]), np.array([[measured_states[idx, 0]], [measured_states[idx, 2]]]))

    kf_x = np.array(kf.measurement_update_cache()[0]).reshape([-1, 5]).T
    sub_plots[0].plot(states[:, 0], states[:, 2])
    sub_plots[0].plot(measured_states[:, 0], measured_states[:, 2])
    sub_plots[0].plot(kf_x[0, :], kf_x[2, :])
    sub_plots[0].legend(["actual states", "measurements", "kf"])

    sub_plots[1].plot(tsim, kf_x[4, :])
    sub_plots[1].legend(["Sanity check g=-9.81"])

    plt.show()