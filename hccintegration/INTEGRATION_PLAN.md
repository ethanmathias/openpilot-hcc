# hCCC Controller Integration Plan for Comma AI Openpilot

## Overview
You need to integrate the hCCC (Human-in-the-Loop Cooperative Cruise Control) controller that was originally designed for BeamNG into the Comma AI openpilot codebase. This document identifies which files need to be updated/merged.

---

## Files to Update/Create

### 1. **Core Controller Module** (NEW FILE)
**Path:** `selfdrive/controls/lib/hccc_controller.py`

**What to do:** Migrate your `hCCC_controller_ICE.py` class into this location.

**Details:**
- Copy the `hCCC` class from `beamngcode/hCCC_controller_ICE.py`
- Adapt it to work with openpilot's CarState and vehicle interfaces instead of BeamNG-specific interfaces
- Remove BeamNG-specific dependencies (beamngpy imports)
- Update the vehicle state access to use openpilot's `CarState` object
- Key changes needed:
  - Replace `self._vehicle` and `self._vehicle_pre` with CarState objects
  - Adapt `run_step()` to accept openpilot CarState and lead vehicle state instead of BeamNG vehicle objects
  - Update sensor polling to use openpilot's messaging system

**Current implementation accesses:**
- `ego_vehicle.state['pos']` → will need `CS.gpsLocation`
- `ego_vehicle.state['vel']` → will need `CS.vEgo`
- `preceding_vehicle` state → will need data from openpilot's lead detection/tracking

---

### 2. **Longitudinal Control Integration** (MODIFY)
**Path:** `selfdrive/controls/lib/longcontrol.py`

**What to do:** Add hCCC as an alternative longitudinal control strategy.

**Current structure:**
- Uses a PID controller for long control
- Has state machine: `off`, `stopping`, `starting`, `pid`

**Needed changes:**
- Import your hCCC controller
- Add a new state or control mode that uses hCCC instead of PID
- Modify the `update()` method to support hCCC as alternative to PID
- Fallback to PID if hCCC isn't available or lead vehicle isn't detected

**Integration point:**
```python
# In the pid control section (around line 80), add:
elif use_hccc_mode:
    output_accel = self.hccc.run_step(CS, lead_vehicle_state)
else:
    # existing PID logic
```

---

### 3. **Controls Main Loop** (MODIFY)
**Path:** `selfdrive/controls/controlsd.py`

**What to do:** Expose hCCC as a control mode option.

**Current structure:**
- Initializes `LongControl` and `LatControl`
- Main control loop calls `self.LoC.update()` for longitudinal control

**Needed changes:**
- Add parameter/setting to enable hCCC mode (e.g., `ENABLE_HCCC`)
- Pass lead vehicle information to LongControl
- Possibly add a new configuration option in CarParams

**Integration point:**
```python
# Around line 120 in the update method:
# Make sure lead vehicle data is available to LongControl.update()
actuators.accel = float(self.LoC.update(
    CC.longActive, CS, long_plan.aTarget, 
    long_plan.shouldStop, pid_accel_limits,
    lead_vehicle_state=sm.get('radarState')  # NEW
))
```

---

### 4. **Longitudinal Planner** (OPTIONAL MODIFY)
**Path:** `selfdrive/controls/lib/longitudinal_planner.py`

**What to do:** Conditionally use hCCC planning instead of MPC planning.

**Current structure:**
- Uses MPC (Model Predictive Control) to generate acceleration targets
- Updates based on radar/model data

**Needed changes:**
- If hCCC mode is enabled, possibly bypass or complement MPC planning
- Feed lead vehicle distance/relative velocity to hCCC
- Could keep MPC as upper safety layer

---

## Key Architectural Differences

### BeamNG Implementation vs Openpilot
| Aspect | BeamNG | Openpilot |
|--------|--------|-----------|
| **Vehicle State** | Direct object access (`vehicle.state['pos']`) | Messages via `SubMaster` |
| **Lead Vehicle** | Direct object reference (`preceding_vehicle`) | Radar/vision tracking (`radarState`) |
| **Control Loop** | Direct vehicle control (`vehicle.control()`) | Message-based `CarControl` |
| **Simulation** | Discrete BeamNG steps | Real-time CAN bus |
| **Sensor Access** | Direct polling (Electrics sensor) | Message subscription |

---

## Data Flow in Openpilot

```
CarState (vehicle telemetry)
    ↓
RadarState (lead vehicle detection)
    ↓
ControlsD (main loop)
    ↓
LongControl (new: hCCC here)
    ↓
CarControl (output to vehicle)
```

---

## Specific Data Mappings Needed

### BeamNG → Openpilot
```python
# BeamNG
ego_speed = math.sqrt(sum(v ** 2 for v in state_ego['vel']))
headway = np.linalg.norm(pos_pre - pos_ego) - 4.5

# Openpilot
ego_speed = CS.vEgo
lead_car = sm['radarState'].leadOne
headway = lead_car.dRel  # radar relative distance
lead_speed = CS.vEgo - lead_car.vRel  # adjust for relative velocity
```

---

## Priority Order for Integration

1. **Phase 1 (Required):** Create `hccc_controller.py` with adapted hCCC class
2. **Phase 2 (Required):** Modify `longcontrol.py` to support hCCC mode
3. **Phase 3 (Required):** Update `controlsd.py` to enable hCCC option
4. **Phase 4 (Optional):** Optimize longitudinal_planner.py interaction
5. **Phase 5 (Testing):** Create test scenarios and validation

---

## Testing Considerations

- Your BeamNG test harness (`hCCC_controller_BeamNG-ICE.py`) will need equivalent for openpilot
- Test with actual vehicle data or simulation (CARLA, BeamNG with openpilot interface)
- Validate against standard openpilot safety limits
- Ensure proper fallback when lead vehicle isn't detected

---

## Summary Table

| File | Action | Complexity | Impact |
|------|--------|-----------|--------|
| `selfdrive/controls/lib/hccc_controller.py` | **CREATE** | High | Core feature |
| `selfdrive/controls/lib/longcontrol.py` | **MODIFY** | Medium | Integrates hCCC |
| `selfdrive/controls/controlsd.py` | **MODIFY** | Low | Enables hCCC option |
| `selfdrive/controls/lib/longitudinal_planner.py` | OPTIONAL | Low | Safety layer |
| `beamngcode/hCCC_controller_ICE.py` | Keep as reference | - | Migration source |
| `beamngcode/hCCC_controller_BeamNG-ICE.py` | Keep as reference | - | Migration source |

