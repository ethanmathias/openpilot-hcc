import numpy as np
from cereal import car


class hCCC:
    """Human-in-the-Loop Cooperative Cruise Control (hCCC) for OpenPilot."""

    def __init__(self, dt=0.1, t_h=1.5, beta=0.65, max_deceleration=-3.0,
                 max_acceleration=3.0, jam=0.0):
        self._dt = dt
        self._t_h = t_h
        self._beta = beta
        self._max_decel = max_deceleration
        self._max_accl = max_acceleration
        self._jam = jam

        self._pre_speed = []
        self._pre_accl = []
        self._feedforward = []
        self._spacing_error = []

        # feedforward filter state
        self.ff_y_prev = 0.0

    def feedforward_no_delay(self, a_lead):
        """Apply the same feedforward filter used in the BeamNG version."""
        th_bar = 1.0
        y = self.ff_y_prev + (self._dt / th_bar) * ((1 - th_bar * self._beta) * a_lead - self.ff_y_prev)
        self.ff_y_prev = y
        return y

    def run_step(self, CS, lead):
        if lead is None or not lead.status:
            return None

        ego_speed = CS.vEgo
        lead_dist = lead.dRel
        lead_speed = ego_speed - lead.vRel

        self._pre_speed.append(lead_speed)
        if len(self._pre_speed) > 1:
            pre_accl_current = (self._pre_speed[-1] - self._pre_speed[-2]) / self._dt
        else:
            pre_accl_current = 0.0
        self._pre_accl.append(pre_accl_current)

        gap = lead_dist - 4.5 - self._jam
        spacing_error = gap / self._t_h - ego_speed
        self._spacing_error.append(spacing_error)

        feedforward_state = self.feedforward_no_delay(self._pre_accl[-1])
        self._feedforward.append(feedforward_state)

        accl_command = (self._beta * (lead_speed - ego_speed) + feedforward_state) * 0.6
        accl_command = np.clip(accl_command, self._max_decel, self._max_accl)

        return float(accl_command)
