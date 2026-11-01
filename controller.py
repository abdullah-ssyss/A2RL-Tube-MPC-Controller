"""AVLite adapter for the extracted Tube MPC solver; no simulator or RL imports."""

from __future__ import annotations

import math

import numpy as np
import osqp
from avlite.c30_control.c31_control_model import AckermannControlCommand
from avlite.c30_control.c32_control_strategy import ControlStrategy

from .parameters import PluginSettings
from .reference import sample_reference
from .tube_mpc import TubeMPCSolver


class A2RLTubeMPCController(ControlStrategy):
    """Track the GlobalPlan/LocalPlan supplied by AVLite with robust lateral MPC."""

    def __init__(self, tj=None, setting=None):
        super().__init__(tj=tj)
        self.setting = setting or PluginSettings
        cfg = {key: getattr(self.setting, key) for key in PluginSettings.__annotations__}
        if not all(math.isfinite(v) for v in cfg.values()):
            raise ValueError("Controller settings must be finite")
        for key in ("wheelbase", "max_steer", "max_steer_rate", "dt", "max_deceleration"):
            if not math.isfinite(cfg[key]) or cfg[key] <= 0:
                raise ValueError(f"{key} must be finite and positive")
        if cfg["horizon_steps"] < 2:
            raise ValueError("horizon_steps must be at least two")
        if any(cfg[key] < 0 for key in ("speed_gain", "max_acceleration", "max_speed",
                                        "tube_boundary_margin", "tube_disturbance_lateral",
                                        "tube_disturbance_heading", "tube_disturbance_steering")):
            raise ValueError("Speed, acceleration, margin and disturbance bounds must be nonnegative")
        self.solver = TubeMPCSolver(
            cfg, wheelbase=cfg["wheelbase"], max_steer=cfg["max_steer"],
            max_steer_rate=cfg["max_steer_rate"],
        )
        self._plan = None
        self.reset()

    def set_plan(self, plan):
        # Use raw plan samples too: GlobalPlan.as_trajectory() can be empty.
        super().set_plan(plan)
        self._plan = plan

    def set_trajectory_tracker(self, tj=None):
        super().set_trajectory_tracker(tj)
        self._plan = None

    def control(self, ego, plan=None, control_dt=None, perception_model=None, sensors=None):
        del perception_model
        if plan is not None:
            self.set_plan(plan)
        dt = self.setting.dt if control_dt is None else float(control_dt)
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("control_dt must be finite and positive")
        try:
            if not all(math.isfinite(v) for v in (ego.x, ego.y, ego.theta, ego.velocity)):
                raise ValueError("Non-finite ego state")
            source = self._plan if self._plan is not None else self.tj
            if source is None:
                raise ValueError("No reference plan")
            path = source.path
            velocity = source.velocity
            if len(path) == 0 and self.tj is not None:
                path, velocity = self.tj.path, self.tj.velocity
            reference = sample_reference(
                path, velocity, getattr(source, "left_boundary_d", []),
                getattr(source, "right_boundary_d", []), ego, self.setting, dt,
            )
            heading_error = math.atan2(math.sin(ego.theta - reference.heading),
                                       math.cos(ego.theta - reference.heading))
            # AVLite's generic ego model has no steering feedback. Use the last
            # commanded road-wheel angle when the bridge does not expose it.
            steering_state = float(getattr(ego, "steering_angle", self._steer))
            if not math.isfinite(steering_state):
                raise ValueError("Non-finite steering feedback")
            result = self.solver.solve(
                lateral_error=reference.lateral_error, heading_error=heading_error,
                steering_state=steering_state, speed=max(ego.velocity, 0.0),
                curvature=reference.curvature, curvature_preview=reference.preview,
                target_offset=0.0, lateral_lower=reference.lower, lateral_upper=reference.upper,
                previous_steer=self._steer, dt=dt,
            )
            if not math.isfinite(result.steering):
                raise RuntimeError("Non-finite MPC output")
            self.last_solver_status = result.status
            self.last_reference = reference
            target_speed = min(reference.speed, self.setting.max_speed)
            acceleration = self.setting.speed_gain * (target_speed - ego.velocity)
            steering = result.steering
        except (ValueError, RuntimeError, np.linalg.LinAlgError, osqp.OSQPException) as exc:
            self.last_solver_status = f"braking: {exc}"
            self.last_reference = None
            self.solver.reset()
            steering = 0.0
            acceleration = -self.setting.max_deceleration
        delta = self.setting.max_steer_rate * dt
        self._steer = float(np.clip(steering, self._steer - delta, self._steer + delta))
        self._steer = float(np.clip(self._steer, -self.setting.max_steer, self.setting.max_steer))
        self.cmd = AckermannControlCommand(
            steer=self._steer,
            acceleration=float(np.clip(acceleration, -self.setting.max_deceleration,
                                       self.setting.max_acceleration)),
        )
        if sensors is not None and sensors.stamp is not None:
            self.cmd.timestamp = float(sensors.stamp)
        return self.cmd

    def reset(self):
        self.solver.reset()
        self._steer = 0.0
        self.last_solver_status = "not_started"
        self.last_reference = None
        self.cmd = AckermannControlCommand()
