# Comma AI Openpilot Architecture: Complete Step-by-Step Explanation

## Table of Contents
1. [High-Level Overview](#high-level-overview)
2. [System Architecture](#system-architecture)
3. [Data Flow Pipeline](#data-flow-pipeline)
4. [Main Control Loop](#main-control-loop)
5. [Key Components Deep Dive](#key-components-deep-dive)
6. [Messaging System](#messaging-system)
7. [Control Stack](#control-stack)
8. [Summary Diagram](#summary-diagram)

---

## High-Level Overview

Openpilot is a **closed-loop vehicle control system** that performs three main functions:

1. **Sense**: Read vehicle state, sensors, and environment data (via CAN bus, cameras, radar)
2. **Plan**: Decide what the vehicle should do next (desired acceleration, steering)
3. **Act**: Send control commands to the vehicle (throttle, brake, steering)

This happens in a **real-time loop at 100 Hz** (every 10ms).

**Key Principle:** Multiple independent processes communicate via **message passing** (not shared memory), making the system modular, safe, and debuggable.

---

## System Architecture

The system is divided into several **independent processes** that run concurrently:

```
┌─────────────────────────────────────────────────────────────┐
│                    OPENPILOT ECOSYSTEM                      │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   SENSING    │  │   PLANNING   │  │   CONTROL    │      │
│  │   (Real HW)  │  │   (Logic)    │  │   (Output)   │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
│         │                 │                   │              │
│    CAN/Camera         ML Model           Vehicle CAN         │
│    Radar/GPS          Tracker            Actuators           │
│         │                 │                   │              │
│         └─────────────────┼───────────────────┘              │
│                           │                                  │
│                  ZMQ Message Bus                             │
│                  (IPC: Inter-Process)                        │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### Main Processes:

| Process | Purpose | Location | Loop Rate |
|---------|---------|----------|-----------|
| **pandad** | CAN communication | `selfdrive/pandad/` | Variable |
| **carD** | Parse CAN to CarState | `selfdrive/car/card.py` | 100 Hz |
| **modelD** | Neural network for trajectory prediction | `selfdrive/modeld/` | 20 Hz |
| **planner** | Plan acceleration/steering | `selfdrive/controls/lib/longitudinal_planner.py` | 20 Hz |
| **controlsd** | Main control loop (THIS IS KEY) | `selfdrive/controls/controlsd.py` | 100 Hz |
| **selfdrived** | State machine & safety | `selfdrive/selfdrived/` | 10 Hz |
| **monitoringd** | Driver monitoring | `selfdrive/monitoring/` | 10 Hz |

---

## Data Flow Pipeline

### Step 0: System Initialization

```python
# selfdrive/controls/controlsd.py - __init__()

Controls.__init__():
  1. Load CarParams (vehicle-specific configuration)
  2. Create CarInterface (brand-specific: Toyota, Honda, etc.)
  3. Create SubMaster for subscribing to messages:
     - carState (vehicle telemetry)
     - modelV2 (ML predictions)
     - longitudinalPlan (desired acceleration)
     - radarState (lead vehicle detection)
     - selfdriveState (enable/disable state)
     - ...11 other messages
  4. Create PubMaster for publishing:
     - carControl (commands to vehicle)
     - controlsState (diagnostics/logging)
  5. Initialize control modules:
     - LongControl (longitudinal PID controller)
     - LatControl (lateral steering controller - PID/torque/angle)
     - VehicleModel (physics for steering)
```

### Step 1: Sensor Data Collection (100 Hz)

```
Vehicle CAN Bus
    ↓
    ├─→ pandad receives raw CAN messages
    │
    └─→ carD parses CAN → builds CarState message
        Contains:
          - vEgo (vehicle speed)
          - aEgo (acceleration from accelerometer)
          - steeringAngleDeg
          - brakePressed
          - throttlePressed
          - cruiseState (cruise control status)
          - gear (P/R/N/D)
          - many other signals...
        
        Publishes: carState message @ 100 Hz
```

**CarState Example:**
```python
CarState {
  vEgo: 20.5,              # 20.5 m/s
  aEgo: 0.3,               # 0.3 m/s² acceleration
  steeringAngleDeg: 2.5,   # 2.5° steering angle
  brakePressed: false,
  throttlePressed: true,
  ...
}
```

### Step 2: Perception & Planning (20 Hz - asynchronous)

**Camera + Model:**
```
Video Stream (20 Hz)
    ↓
    └─→ modelD (neural network in modelD.so)
        Predicts:
          - Lane positions (left/right boundaries)
          - Desired path curvature
          - Vehicle trajectory for next 5 seconds
        
        Publishes: modelV2 message
```

**Radar:**
```
Radar Points (variable rate)
    ↓
    └─→ radarD or internal tracking
        Detects:
          - Lead vehicle distance (dRel)
          - Lead vehicle relative velocity (vRel)
          - Multiple objects around vehicle
        
        Publishes: radarState message
```

**Longitudinal Planner (20 Hz):**
```python
LongitudinalPlanner.update(sm):
  1. Read current vehicle state (CarState)
  2. Read lead vehicle info (radarState)
  3. Read path prediction (modelV2)
  4. Run MPC (Model Predictive Control) solver
  5. Compute desired acceleration profile for next 5 seconds
  6. Publish: longitudinalPlan message
     - aTarget (acceleration target)
     - shouldStop (boolean)
     - hasLead (is there a car ahead?)
```

### Step 3: Main Control Loop (100 Hz) - THE BRAIN

This happens in **`controlsd.py::Controls.run()`**:

```python
while True:
    # Step A: Update inputs
    self.sm.update(15)  # Wait for new messages (15ms timeout)
    
    # Step B: Run control logic
    CC, lac_log = self.state_control()  # <-- WHERE DECISIONS HAPPEN
    
    # Step C: Publish outputs
    self.publish(CC, lac_log)  # Send carControl to vehicle
    
    # Step D: Rate limiting
    rk.monitor_time()  # Enforce 100 Hz rate
```

---

## Main Control Loop

### The Heart: `state_control()` Method

This is where the magic happens. Let me break it down:

```python
def state_control(self):
    # 1. READ INPUTS
    CS = self.sm['carState']           # Current vehicle state
    long_plan = self.sm['longitudinalPlan']  # Desired acceleration
    model_v2 = self.sm['modelV2']      # Lane prediction
    lp = self.sm['liveParameters']     # Calibration
    
    # 2. UPDATE VEHICLE MODEL
    # Recalculate steering dynamics based on new parameters
    self.VM.update_params(lp.stiffnessFactor, lp.steerRatio)
    
    # Calculate current road curvature from steering angle
    steer_angle_without_offset = CS.steeringAngleDeg - lp.angleOffsetDeg
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo)
    
    # 3. CREATE OUTPUT MESSAGE
    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled  # Am I supposed to be driving?
    
    # 4. CHECK IF STEERING IS ALLOWED
    standstill = abs(CS.vEgo) <= 0.3 or CS.standstill
    CC.latActive = (CC.enabled and 
                    not CS.steerFault and 
                    (standstill == false or steerAtStandstill))
    
    # 5. CHECK IF ACCELERATION IS ALLOWED
    CC.longActive = (CC.enabled and 
                     not overrideLongitudinal and 
                     openpilotLongitudinalControl)
    
    # 6. RESET IF NOT ACTIVE
    if not CC.latActive:
        self.LaC.reset()  # Clear steering controller memory
    if not CC.longActive:
        self.LoC.reset()  # Clear throttle/brake controller memory
    
    # 7. RUN LONGITUDINAL CONTROL (THROTTLE/BRAKE)
    # ───────────────────────────────────────────
    pid_accel_limits = self.CI.get_pid_accel_limits(CS.vEgo)
    actuators.accel = self.LoC.update(
        CC.longActive,              # Is long control active?
        CS,                         # Current state
        long_plan.aTarget,          # Desired acceleration from planner
        long_plan.shouldStop,       # Should we stop?
        pid_accel_limits            # Min/max acceleration allowed
    )
    # Output: accel value from -3.0 (brake) to +3.0 (throttle) m/s²
    
    # 8. RUN LATERAL CONTROL (STEERING)
    # ─────────────────────────────────
    # Smoothly interpolate desired curvature to avoid jumps
    new_desired_curvature = (model_v2.action.desiredCurvature 
                             if CC.latActive else self.curvature)
    self.desired_curvature, curvature_limited = clip_curvature(...)
    
    # Update steering controller
    steer, steeringAngleDeg, lac_log = self.LaC.update(
        CC.latActive,                # Is lat control active?
        CS,                          # Current state
        self.VM,                     # Vehicle model
        lp,                          # Live parameters
        self.desired_curvature,      # Target curvature
        lat_delay                    # Sensor latency
    )
    # Output: steering torque or angle (depends on car type)
    
    # 9. PACKAGE OUTPUTS
    actuators.accel = accel_value
    actuators.torque = steer
    actuators.steeringAngleDeg = steeringAngleDeg
    
    # 10. SAFETY CHECK: No NaN/Inf values
    for field in actuators:
        if not math.isfinite(getattr(actuators, field)):
            cloudlog.error(f"Bad value: {field}")
            setattr(actuators, field, 0.0)
    
    return CC, lac_log
```

---

## Key Components Deep Dive

### 1. LONGITUDINAL CONTROL (Throttle/Brake)

**File:** `selfdrive/controls/lib/longcontrol.py`

**Purpose:** Calculate throttle/brake command to achieve target acceleration

**Architecture:**

```python
class LongControl:
    def __init__(self, CP):
        # PID controller tuning parameters from CarParams
        self.pid = PIDController(
            kp=(kpBP, kpV),  # Proportional gains at different speeds
            ki=(kiBP, kiV)   # Integral gains at different speeds
        )
        self.long_control_state = LongCtrlState.off
    
    def update(self, active, CS, a_target, should_stop, accel_limits):
        """
        Inputs:
          active: bool - Is control enabled?
          CS: CarState - Current vehicle state (speed, accel, etc)
          a_target: float - Desired acceleration from planner (m/s²)
          should_stop: bool - Emergency stop signal
          accel_limits: [min, max] - Safety limits
        
        Returns:
          output_accel: float - Acceleration command (-3 to +3 m/s²)
        """
        
        # State Machine with 4 states:
        # ┌─────────┐
        # │   OFF   │ ← Default, no control
        # └────┬────┘
        #      ↓
        # ┌─────────┐
        # │STOPPING │ ← Gradual deceleration to stop
        # └────┬────┘
        #      ↓
        # ┌─────────┐
        # │STARTING │ ← Gentle acceleration from stop
        # └────┬────┘
        #      ↓
        # ┌─────────┐
        # │  PID    │ ← Normal cruise control
        # └─────────┘
        
        if not active:
            return 0.0  # No control
        
        if state == STOPPING:
            # Gradual deceleration
            output = apply_stopping_decel_rate()
        
        elif state == STARTING:
            # Gentle acceleration from standstill
            output = CP.startAccel  # Usually 1.0 m/s²
        
        else:  # PID state
            # Error between desired and actual acceleration
            error = a_target - CS.aEgo
            
            # PID calculates: P*error + I*integral(error) + D*derivative(error)
            output = self.pid.update(
                error,
                speed=CS.vEgo,
                feedforward=a_target  # Feedforward helps MPC predictions
            )
        
        # Clip to safety limits
        output = clip(output, accel_limits[0], accel_limits[1])
        
        return output
```

**Example Scenario:**
```
Desired acceleration (from MPC): +1.5 m/s²
Current acceleration (from IMU): +1.2 m/s²
Error: +0.3 m/s²

PID output: 
  - P term: 0.3 * kp = 0.15
  - I term: 0.5 * ki = 0.1  (accumulated over time)
  - D term: -0.1 * kd = 0.0
  → Total: 0.25 m/s² extra

Final command: 1.2 + 0.25 = 1.45 m/s² ✓
```

---

### 2. LATERAL CONTROL (Steering)

**File:** `selfdrive/controls/lib/latcontrol.py` (base class)

**Variants:** 
- `latcontrol_pid.py` - PID steering (most cars)
- `latcontrol_torque.py` - Torque steering (newer cars)
- `latcontrol_angle.py` - Direct angle control (Tesla, etc)

**Purpose:** Calculate steering command to follow the lane

**High-Level:**

```python
class LatControl:
    def update(self, active, CS, VM, params, desired_curvature, lat_delay):
        """
        Inputs:
          active: bool - Is steering enabled?
          CS: CarState - Current steering angle, speed
          VM: VehicleModel - Physics model of steering
          desired_curvature: float - Target curvature from lane detection
          lat_delay: float - Sensor processing delay (20-50ms)
        
        Returns:
          steer_output: float - Steering torque (Nm) or angle (deg) or rate
          steeringAngleDeg: float - Command steering angle
        """
        
        if not active:
            return 0.0
        
        # Account for sensor latency
        # Predict where car will be in lat_delay time
        future_position = CS.lateralError + CS.lateralVelocity * lat_delay
        
        # Calculate steering correction
        # PID: lateral position error + lateral velocity error
        pid_output = self.pid.update(
            error=future_position,
            speed=CS.vEgo
        )
        
        # Convert to vehicle command
        # Different cars need different formats
        if torque_steering:
            steer_cmd = torque_to_nm(pid_output)
        elif angle_steering:
            steer_cmd = pid_output  # Direct steering angle
        else:
            steer_cmd = pid_output  # Steering rate
        
        return steer_cmd
```

**Key Insight:** Steering uses **predictive control** - it predicts where the car will be in 20-50ms due to sensor lag and steers accordingly.

---

### 3. VEHICLE MODEL

**File:** `opendbc/car/vehicle_model.py`

**Purpose:** Calculate steering/physics relationships

```python
class VehicleModel:
    def calc_curvature(self, steer_angle, v_ego):
        """
        Convert steering angle to path curvature
        Uses bicycle model: curvature = tan(angle) / wheelbase
        """
        # steer_angle (radians) → curvature (1/meters)
        return tan(steer_angle) / wheelbase
```

---

### 4. LONGITUDINAL PLANNER

**File:** `selfdrive/controls/lib/longitudinal_planner.py`

**Purpose:** Generate smooth acceleration trajectory based on:
- Current speed
- Lead vehicle distance and speed  
- Coast detection (downhill/uphill)
- Model prediction (predict lead behavior)

**Algorithm: MPC (Model Predictive Control)**

```python
class LongitudinalPlanner:
    def update(self, sm):
        # Read all inputs
        v_ego = sm['carState'].vEgo
        lead = sm['radarState'].leadOne
        model = sm['modelV2']
        
        # Run MPC solver
        # MPC optimizes acceleration over next 5 seconds
        # to: 1) Follow lead car, 2) Minimize jerk, 3) Stay within limits
        
        self.mpc.run(
            x0=v_ego,
            lead_distance=lead.dRel,
            lead_speed=lead.vRel,
            traffic_mode=sm['selfdriveState'].experimentalMode
        )
        
        # Extract planned accelerations
        self.output_a_target = self.mpc.solution[0]
        self.output_should_stop = v_ego < 0.3 and self.output_a_target < 0
        
        return longitudinalPlan(
            aTarget=self.output_a_target,
            shouldStop=self.output_should_stop,
            hasLead=lead is not None
        )
```

---

## Messaging System

Openpilot uses **Cereal** (a capnproto library) for efficient inter-process communication.

### Message Schema (Defined in `.capnp` files)

```python
# cereal/car.capnp defines CarControl schema:
CarControl {
  enabled: Bool              # Master enable
  latActive: Bool            # Steering active?
  longActive: Bool           # Throttle/brake active?
  
  actuators {
    torque: Float32          # Steering torque (Nm)
    accel: Float32           # Acceleration command (m/s²)
    steeringAngleDeg: Float32
    longControlState: LongControlState  # off/stopping/starting/pid
  }
  
  hudControl {
    setSpeed: Float32
    leadVisible: Bool
    visualAlert: AlertType
  }
}
```

### Message Flow in Code

```python
# PUBLISH a message
import cereal.messaging as messaging

pm = messaging.PubMaster(['carControl'])

# Create message
msg = messaging.new_message('carControl')
msg.carControl.enabled = True
msg.carControl.actuators.accel = 1.5

# Send it
pm.send('carControl', msg)

# ───────────────────────────────────────────

# SUBSCRIBE to messages
sm = messaging.SubMaster(['carState', 'radarState'])

while True:
    sm.update(timeout_ms=15)  # Wait for messages
    
    if sm.updated['carState']:
        print(f"Speed: {sm['carState'].vEgo} m/s")
    
    if sm.updated['radarState']:
        print(f"Lead car distance: {sm['radarState'].leadOne.dRel} m")
```

---

## Control Stack

### The Full Data Flow Visualization

```
┌─────────────────────────────────────────────────────────────────┐
│                     CONTROLSD.PY (100 Hz)                       │
└─────────────────────────────────────────────────────────────────┘
                             ▲                 │
                             │                 │
                      ┌──────┴─────────────────┴───────┐
                      │                                 │
                      ▼                                 ▼
            ┌────────────────────┐        ┌────────────────────┐
            │  LATERAL CONTROL   │        │ LONGITUDINAL PLAN  │
            │  (Steering PID)    │        │  (MPC Solver)      │
            │                    │        │                    │
            │ Input: Lane curve  │        │ Input: Lead car    │
            │ Output: Torque     │        │ Output: Accel      │
            └────────────────────┘        └────────────────────┘
                      ▲                                 ▲
                      │                                 │
        ┌─────────────┴─────────────┬─────────────────┴──────────┐
        │                           │                            │
        ▼                           ▼                            ▼
    ┌─────────┐            ┌──────────────┐         ┌──────────────┐
    │ModelV2  │            │  CarState    │         │ RadarState   │
    │(20Hz)   │            │  (100Hz)     │         │  (variable)  │
    │         │            │              │         │              │
    │Lane     │            │Steering Ang  │         │Lead distance │
    │Curve    │            │Vehicle speed │         │Lead velocity │
    │Traj     │            │Acceleration  │         │Position      │
    └─────────┘            └──────────────┘         └──────────────┘
        ▲                           ▲                            ▲
        │                           │                            │
        └───────────────────────────┴────────────────────────────┘
                           │
                           ▼
                  ┌──────────────────┐
                  │   Vehicle CAN    │
                  │  (Cameras/Radar) │
                  └──────────────────┘
                           ▲
                           │
                  ┌──────────────────┐
                  │  Physical Car    │
                  │                  │
                  │  Wheels/Engine   │
                  └──────────────────┘
```

---

## Summary Diagram: The 100Hz Loop

Here's what happens every 10 milliseconds:

```
TIME: 0ms
├─ controlsd wakes up
├─ sm.update(15)  ← Wait for new messages (up to 15ms)
│
TIME: ~5-10ms (messages arrive)
├─ Read CarState (vehicle telemetry)
├─ Read ModelV2 (lane prediction)
├─ Read LongitudinalPlan (desired acceleration)
├─ Read LiveParameters (calibration)
├─ Read RadarState (lead vehicle)
│
TIME: ~10-15ms
├─ Run LATERAL control:
│  ├─ Calculate steering error
│  ├─ Run PID loop
│  └─ Output: steering torque
│
├─ Run LONGITUDINAL control:
│  ├─ Compare desired vs actual acceleration
│  ├─ Run PID loop
│  └─ Output: throttle/brake command
│
TIME: ~16-17ms
├─ Publish CarControl message
│  ├─ actuators.accel
│  ├─ actuators.torque
│  ├─ actuators.steeringAngleDeg
│  └─ other fields
│
├─ Publish ControlsState (logging)
│
TIME: ~18-19ms
├─ rk.monitor_time() ← Enforce 100Hz (sleep if too fast)
│
TIME: ~20ms (next iteration)
└─ Loop repeats...
```

---

## Key Takeaways

### The "Brain" Flow
1. **Sense** → Read sensors via CarState/RadarState messages
2. **Perceive** → ModelD identifies lanes, leads
3. **Plan** → LongitudinalPlanner decides acceleration trajectory  
4. **Decide** → ControlsD runs LongControl and LatControl
5. **Act** → Send CarControl with steering/accel commands
6. **Loop** → Repeat 100x per second

### Safety Architecture
- **Watchdog timers** - If a process dies, system disables
- **State machine** - Can only transition through safe states
- **Latency compensation** - 20-50ms sensor lag handled
- **Limits checking** - All commands clipped to safe ranges
- **Fallback** - Defaults to safe values (0 accel, 0 steering)

### Why ZMQ Messaging?
- **Decoupled** - Processes don't need to know about each other
- **Testable** - Can record/replay messages
- **Safe** - One crashed process doesn't crash others
- **Efficient** - Shared memory would be slower and less safe
- **Debuggable** - Can inspect messages at any point

---

## File Reference Quick Guide

```
selfdrive/controls/
├─ controlsd.py              ← MAIN LOOP (100 Hz controller)
├─ lib/
│  ├─ longcontrol.py         ← Throttle/brake control (PID)
│  ├─ latcontrol.py          ← Steering control base class
│  ├─ latcontrol_pid.py      ← PID steering variant
│  ├─ latcontrol_torque.py   ← Torque steering variant
│  ├─ longitudinal_planner.py ← MPC acceleration planning
│  └─ drive_helpers.py       ← Helper functions
└─ tests/                    ← Unit tests

selfdrive/car/
├─ car_specific.py           ← Safety checks
├─ card.py                   ← CAN parsing loop
├─ [brand]/
│  ├─ interface.py           ← CarInterface implementation
│  ├─ carstate.py            ← CAN → CarState parser
│  └─ carcontroller.py       ← CarControl → CAN sender

cereal/
└─ car.capnp                 ← Message schema definitions
```

This is the foundation of how openpilot controls vehicles! 🚗
