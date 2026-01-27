import math
import numpy as np

class hCCC:
    """
    Human-in-the-Loop Cooperative Cruise Control (hCCC) Controller for BeamNG.
    Returns normalized throttle and brake commands (0.0 to 1.0).
    Positive acceleration -> throttle
    Negative acceleration -> brake
    """

    def __init__(self, ego_vehicle, preceding_vehicle, dt=0.1, t_h=1.5, beta=0.65, 
                 max_deceleration=-3, max_acceleration=3, jam=0):
        self._dt = dt
        self._beta = beta
        self._t_h = t_h
        self._max_decel = max_deceleration
        self._max_accl = max_acceleration
        self._jam = jam

        self._vehicle = ego_vehicle
        self._vehicle_pre = preceding_vehicle

        self._speed_ego = []
        self._accl_ego = []
        self._accl_pre = []
        self._speed_commad = []
        self._pre_accl = []
        self._feedforward = []
        self._pre_speed = []
        self._pre2_speed = []
        self._spacing_error = []

        # feedforward filter states (NO CLASS)
        self.ff_y_prev = 0.0

    def feedforward_no_delay(self, a_lead):
        """
        Apply feedforward filter (Eq.7) without delay.
        """
        th_bar = 1.0

        y = self.ff_y_prev + (self._dt / th_bar) * ((1 - th_bar * self._beta) * a_lead - self.ff_y_prev)

        self.ff_y_prev = y

        return y

    def run_step(self):
        """
        Compute a control step to maintain a safe distance.
        Returns (throttle, brake):
          - throttle in [0.0, 1.0]
          - brake in [0.0, 1.0]
        """
        
        # Poll sensors to get updated vehicle states
        self._vehicle_pre.poll_sensors()
        self._vehicle.poll_sensors()
        
        ego_state = self._vehicle.state
        preceding_state = self._vehicle_pre.state

        current_speed = math.sqrt(sum(v ** 2 for v in ego_state['vel']))
        pre_speed_current = math.sqrt(sum(v ** 2 for v in preceding_state['vel']))

        self._pre_speed.append(pre_speed_current)

        gap = (preceding_state['pos'][0] - ego_state['pos'][0]) - 4.5 - self._jam
        spacing_err = gap / self._t_h - current_speed
        
        self._spacing_error.append(spacing_err)

        if len(self._pre_speed) > 1:
            pre_accl_current = (self._pre_speed[-1] - self._pre_speed[-2]) / self._dt
        else:
            pre_accl_current = 0.0
        self._pre_accl.append(pre_accl_current)

        state = self.feedforward_no_delay(self._pre_accl[-1])
        self._feedforward.append(state)

        accl_command = (self._beta * (pre_speed_current - current_speed) + self._feedforward[-1]) * 0.6

        return accl_command