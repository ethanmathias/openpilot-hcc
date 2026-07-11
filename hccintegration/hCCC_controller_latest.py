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

        self._speed_ego = []
        self._accl_ego = []
        self._accl_pre = []
        self._speed_commad = []
        self._pre_accl = []
        self._feedforward = []
        self._pre_speed = []
        self._pre2_speed = []
        self._spacing_error = []

        # Parameters for the transfer function
        tau = 0.12  # given
        beta = 0.65  # given
        th_bar = 1.5  # given
        discretize_method: str = "tustin"

        # ---- Build continuous-time transfer function ----
        # F(s) = (1 + tau * s - beta * th_bar) / (1 + s * th_bar)
        # Numerator: (tau * s) + (1 - beta * th_bar)
        # Denominator: (th_bar * s) + 1

        numerator = [tau, 1 - beta * th_bar]  # [tau, constant term]
        denominator = [th_bar, 1.0]  # [th_bar, constant term]

        Fc = signal.TransferFunction(numerator, denominator)

        # ---- Discretize ----
        Fd = Fc.to_discrete(self._dt, method=discretize_method)

        self.b = np.asarray(Fd.num).squeeze()
        self.a = np.asarray(Fd.den).squeeze()

        # Filter state (Direct-form via lfilter zi)
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

        current_speed = math.sqrt(sum(v ** 2 for v in ego_state['vel']))
        pre_speed_current = math.sqrt(sum(v ** 2 for v in preceding_state['vel']))

        self._pre_speed.append(pre_speed_current)

        gap = np.linalg.norm(
            np.array(preceding_state['pos']) - np.array(ego_state['pos'])
        ) - 4.5 - self._jam
        spacing_err = gap / self._t_h - current_speed

        self._spacing_error.append(spacing_err)

        if len(self._pre_speed) > 1:
            pre_accl_current = (self._pre_speed[-1] - self._pre_speed[-2]) / self._dt
        else:
            pre_accl_current = 0.0
        self._pre_accl.append(pre_accl_current)

        state = self._filter_step(self._pre_accl[-1])
        self._feedforward.append(state)

        gap_rate = pre_speed_current - current_speed
        #v_des = current_speed + 0.4 * spacing_err + self._beta * gap_rate + self._feedforward[-1]
        v_des = current_speed + self._beta * gap_rate + self._feedforward[-1]

        v_des = max(v_des, 0.0)          # speed cannot be negative
        # ------------------------------------------------------------
        # Use PID to track the desired speed -> get throttle/brake
        # ------------------------------------------------------------
        accl_command = self._pid.run_step(v_des, current_speed)   # in [-1, 1]

        if accl_command >= 0:
            throttle = accl_command
            brake = 0.0
        else:
            throttle = 0.0
            brake = -accl_command

        return throttle, brake