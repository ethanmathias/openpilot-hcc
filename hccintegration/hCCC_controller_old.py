import math
import numpy as np
from scipy import signal

class PIDLongitudinal:
    """
    A simplified version of the PIDLongitudinalController from vehicle_controller.py
    """
    def __init__(self, K_P, K_I, K_D, dt):
        self._k_p = K_P
        self._k_i = K_I
        self._k_d = K_D
        self._dt = dt
        self._error_buffer = []

    def run_step(self, target_speed, current_speed):
        error = target_speed - current_speed
        self._error_buffer.append(error)

        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2])
            _ie = sum(self._error_buffer) * self._dt
        else:
            _de = 0.0
            _ie = 0.0

        # Output is a normalised acceleration request in [-1, 1]
        accel_cmd = self._k_p * error + self._k_d * _de + self._k_i * _ie

        if 0.5 < target_speed < 10.0:
            accel_cmd = accel_cmd * target_speed / 10.0
        return np.clip(accel_cmd, -1.0, 1.0)
    
class hCCC:
    """
    Human-in-the-Loop Cooperative Cruise Control (hCCC) Controller for BeamNG.
    Returns normalized throttle and brake commands (0.0 to 1.0).
    Positive acceleration -> throttle
    Negative acceleration -> brake
    """

    def __init__(
        self, 
        ego_vehicle, 
        preceding_vehicle, 
        dt=0.1, 
        t_h=1.5, 
        beta=0.65, 
        max_deceleration=-3, 
        max_acceleration=3, 
        jam=0,
        pid_K_P=0.35,      # Proportional gain
        pid_K_I=0.05,     # Integral gain (helps eliminate steady-state error)
        pid_K_D=0.2,      # Derivative gain
        ):

        self._dt = dt
        self._beta = beta
        self._t_h = t_h
        self._max_decel = max_deceleration
        self._max_accl = max_acceleration
        self._jam = jam

        self._vehicle = ego_vehicle
        self._vehicle_pre = preceding_vehicle

        self._pre_speed = []


        # ---------- Filter (same as original) ----------
        tau = 0.12
        beta = 0.65
        th_bar = 1.5
        numerator = [tau, 1 - beta * th_bar]
        denominator = [th_bar, 1.0]
        Fc = signal.TransferFunction(numerator, denominator)
        Fd = Fc.to_discrete(self._dt, method='tustin')
        self.b = np.asarray(Fd.num).squeeze()
        self.a = np.asarray(Fd.den).squeeze()
        self.z = np.zeros(max(len(self.a), len(self.b)) - 1)
    
        # -----------------------------
        # PID controller for speed tracking
        # -----------------------------
        self._pid = PIDLongitudinal(
            K_P=pid_K_P,
            K_I=pid_K_I,
            K_D=pid_K_D,
            dt=self._dt
        )

    def reset(self):
        """Reset the filter state."""
        self.z[:] = 0.0
        self._pre_speed.clear()
        self._pid._error_buffer.clear()

    def _filter_step(self, u: float) -> float:
        """Apply F(s) to input u and return filtered output."""
        y, self.z = signal.lfilter(self.b, self.a, [u], zi=self.z)
        return float(y[0])

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

        v_ego = float(np.linalg.norm(ego_state['vel']))
        v_pre = float(np.linalg.norm(preceding_state['vel']))

        # Acceleration of preceding2 (simple difference)
        if len(self._pre_speed) > 0:
            ap = (v_pre - self._pre_speed[-1]) / self._dt
        else:
            ap = 0.0
        self._pre_speed.append(v_pre)

        ff = self._filter_step(ap)

        gap_rate = v_pre - v_ego
        v_des = v_ego + self._beta * gap_rate + ff
        v_des = max(v_des, 0.0)          # speed cannot be negative
        # ------------------------------------------------------------
        # Use PID to track the desired speed -> get throttle/brake
        # ------------------------------------------------------------
        accel_cmd = self._pid.run_step(v_des, v_ego)   # in [-1, 1]

        # Saturation
        accel_cmd = float(np.clip(accel_cmd, -1.0, 1.0))

        return accel_cmd
