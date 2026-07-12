# HC3 in-car field test, 2026-06-14

**Vehicle:** 2023 Kia Sportage (NQ5, CAN-FD), base trim.
**Devices:** two Comma 3X, ego on the car harness, lead USB-powered in the cabin
replaying scenario 48.
**Goal:** first in-car validation of the full HC3 loop, with V2V lead data
driving the ego's hCCC longitudinal control on a real car.

## Outcome

Every part of HC3 was validated except the final link, openpilot commanding the
car's longitudinal actuation. That link failed because this Sportage lacks the
Smart Cruise Control (SCC) package openpilot needs to inject acceleration.
Decision: switch to a radar-SCC supported car. All other work is car-agnostic and
carries over.

## What worked

- V2V transport: lead to relay (on ego) to ego subscriber, every run, 2000/2000
  packets, 0% loss at 50 Hz, inter-arrival median about 20 ms (100 ms staleness
  limit). Matched the bench result, in the car.
- Clock sync: solved the 500 ms V2V skew problem in the field (sync lead to ego,
  then compensate for SSH latency). Held under the limit through the session.
- Orchestration and data: `field_test.py` preflight (14/14), remote scenario
  start/stop, and automatic artifact collection all worked.
- Car recognition: brought the unsupported Sportage out of dashcam mode (see
  below) to a valid car interface.
- Engagement and hCCC: openpilot engaged, ran its longitudinal PID
  (`long_state=pid`), and hCCC produced smooth, correctly-signed acceleration
  commands (for example a -0.1 to -2.0 m/s2 ramp tracking the lead). The lead-lag
  feedforward ran stably on real hardware. The control logic is sound.

## Car-recognition detour (resolved)

openpilot first booted in dashcam mode ("car unrecognized"). The car returned
valid NQ5 firmware for 14 ECUs but no radar module (no `fwdRadar` 0x7d0), so it
did not match the radar-based Sportage fingerprint. Worked around on-device:

1. Forced the fingerprint (`FINGERPRINT=KIA_SPORTAGE_5TH_GEN`, `SKIP_FW_QUERY=1`
   in `launch_env.sh`), which cleared dashcam but threw `canError` ("Unknown
   Vehicle Variant").
2. Added `HyundaiFlags.CAMERA_SCC` to the Sportage platform in opendbc, which
   cleared `canError` and produced a valid car interface.

These are device-only edits, to be reverted before switching cars.

## Blocker: longitudinal actuation

With everything configured, openpilot engaged and commanded acceleration but the
car never responded. Isolated with joystick mode (which bypasses hCCC, controlsd,
and V2V for direct keyboard gas/brake): the car still did not move. So the
failure is in the openpilot-to-car actuation layer, not HC3.

Diagnostics confirmed the software was configured correctly:

| Field | Value | Meaning |
|---|---|---|
| `openpilotLongitudinalControl` | True | openpilot set to command gas/brake itself |
| `pcmCruise` | False | openpilot owns cruise, not the factory PCM |
| `minEnableSpeed` | -1.0 | openpilot imposes no speed floor |
| `radarUnavailable` | True | no radar |
| `safetyModel` | hyundaiCanfd | correct |

Since openpilot's own `minEnableSpeed` is -1, the 20 mph engage floor we hit came
from the car, not openpilot. A 20 mph floor with no stop-and-go is the signature
of standard, non-adaptive cruise control. Combined with no radar and a base-trim
marker (firmware reported manual climate control), the conclusion is that this
vehicle does not have the SCC (adaptive cruise) hardware openpilot requires. With
no SCC there is no acceleration-command interface to inject into: openpilot
computes commands that have nowhere to go. Not fixable on this car.

## Decision and next steps

- Switch to a supported car with radar-SCC (a separate radar module), which
  sidesteps both the camera-SCC injection problem and the missing-SCC problem.
  openpilot longitudinal works out of the box on radar-SCC Hyundai/Kia.
- The HC3 stack is car-agnostic (grep-confirmed: no car-specific references in the
  controller, longcontrol hook, V2V, or tooling). Switching cars needs no code
  changes.
- Before the swap, undo the three Sportage-only device hacks: remove the forced
  fingerprint from `launch_env.sh`, restore `values.py` from its backup, and
  revert the joystick guard bypass plus clear `JoystickDebugMode`.
- Then bolt the device to the new car with the correct harness, let it fingerprint
  natively (verify `dashcam=False`, `opLong=True`), re-set
  `AlphaLongitudinalEnabled`, re-preflight, and repeat the engage test.

## Bottom line

The day proved the hard parts of HC3 (V2V transport, timing, the hCCC controller,
and the full orchestration) work on a real car. The one thing that blocked a
complete drive was a vehicle hardware limitation (no SCC), not a flaw in the
system. The fix is a car swap, not a redesign.
