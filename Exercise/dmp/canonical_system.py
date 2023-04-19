from __future__ import division, print_function

from typing import List, Union

import numpy as np


class CanonicalSystem(object):
    def __init__(self, alpha: float):
        self.alpha = alpha
        self.step_vectorized = np.vectorize(self.step, otypes=[float])
        self.reset()
        self.x = 1.0

    def step(self, dt: float, tau: float):
        """
        Solve the canonical system at next time step t+dt.

        Parameters
        ----------
        dt : float
            Time step.
        tau : float
            Temporal scaling factor.
        """
        x = self.x
        self.x += -self.alpha * x / tau * dt  # dx/dt = alpha * x / tau
        return x

    def rollout(self, t: List[float], tau: Union[float, List[float]]):
        """
        Solve the canonical system.

        Parameters
        ----------
        t : array_like
            Time points for which to evaluate the integral.
        tau : array_like
            Temporal scaling factor (scalar constant or same length as t).
        """
        self.reset()
        return self.step_vectorized(np.gradient(t), tau)

    def reset(self):
        self.x = 1.0

    # def rollout(self, ts, tau):
    #     from scipy.integrate import odeint

    #     def cs(x, t, tau, alpha):
    #         dxdt = -alpha * x * tau
    #         return dxdt

    #     x0 = 1.0
    #     sol = odeint(cs, x0, ts, args=(tau, self.alpha))
    #     return sol[:,0]
