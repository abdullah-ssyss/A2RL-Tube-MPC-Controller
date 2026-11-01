# A2RL Tube MPC Controller for AVLite

Standalone community **control** plugin extracted from
[AA05](https://github.com/OsamaMIT/AA05). Copy the contents of this directory
to the root of a new GitHub repository named `avlite_a2rl_tube_mpc`.

This directory contains only the lateral Tube MPC solver, its AVLite control
adapter, parameters and tests. It has no imports from `chrono_a2rl`, no RL
environment, Gymnasium, SB3, Chrono, track dataset or planner. The host AVLite
stack supplies a `GlobalPlan`, `LocalPlan` or `TrajectoryTracker` and ego state.

## Algorithm and integration

`tube_mpc.py` is extracted unchanged from
`src/chrono_a2rl/control/tube_mpc.py` in AA05: linear lateral-error model with
steering actuator lag, nominal QP using OSQP, ancillary LQR feedback and an
iterated disturbance tube used to tighten constraints. It is a lateral MPC,
not a nonlinear vehicle dynamics optimizer.

`controller.py` adapts the supplied AVLite plan instead of loading a fixed
raceline from disk. It projects ego onto the plan, previews curvature by arc
length and commands steering in radians. A small proportional speed tracker
returns bounded acceleration in m/s2 from the plan's velocity samples; speed
is not part of the MPC optimization. There is no throttle/brake/gear interface.
The bridge must accept `AckermannControlCommand` and provide localized ego
pose and speed in the same coordinate system as the plan.

Global plans can supply signed `left_boundary_d` / `right_boundary_d`, relative
to the supplied path. The adapter uses the most restrictive sampled corridor
over the horizon. For local plans or trackers without boundaries, the default
assumption is 4 m on each side; configure this explicitly for your use case.
This assumption is not a measurement of track limits. Curvature and boundary
sampling are approximations; collision avoidance belongs to the host planner.

On missing/invalid plans or an infeasible solve, the adapter requests bounded
deceleration and gradually returns steering to zero. It does not switch to
pure pursuit. Check `last_solver_status`, `last_reference` and
`solver.last_result` for diagnostics. Generic AVLite ego state has no steering
feedback; the adapter uses `ego.steering_angle` when available, otherwise the
previous command. `reset()` clears control and solver memory, preserving the
current reference plan.

## Install locally

Target API: AVLite 0.5.3 through 0.5.x. From this directory:

```powershell
python -m pip install -r requirements.txt
```

Keep this folder named `avlite_a2rl_tube_mpc`. Add it to your active AVLite
profile using an absolute path (replace the example):

```yaml
c69_apps:
  c62_community_plugins:
    avlite_a2rl_tube_mpc: C:/path/to/avlite_a2rl_tube_mpc
  c62_load_plugins: true
c40_execution:
  c40_controller: A2RLTubeMPCController
  c40_control_dt: 0.02
plugins:
  avlite_a2rl_tube_mpc:
    closed_path: false
    wheelbase: 3.115
    max_steer: 0.38
    max_steer_rate: 1.5
```

Merge these entries with your profile; select your own bridge and planners.
Set `closed_path: true` only if every incoming plan describes a full circuit.
Keep it false for rolling local plans. Missing speed samples mean a zero
target speed. Parameters are exposed through AVLite's settings interface;
`settings.py` and the controller share the class in `parameters.py`.

Vehicle defaults are A2RL-style research estimates from AA05, not calibrated
private vehicle data. No full-speed A2RL lap or real-car validation is claimed.

## Tests from this directory

```powershell
python -m pip install -r requirements-dev.txt
python -m pytest tests -q
```

Tests copy this plugin outside the source repository, load it through AVLite
and exercise the OSQP controller without the training package.

## Publish

Commit this directory's contents as the new repository root. Choose a license
consistent with the source code's rights before public redistribution; no
license grant is added by this extraction. Retain the AA05 attribution above.
Replace placeholders in `registry-entry.yaml`, create the referenced tag and
submit that entry under `plugins:` in `AV-Lab/avlite-community-plugins`.
The registry category is `ControlStrategy`. No GitHub publication is performed
by this directory. Do not commit a virtual environment or Python caches.
