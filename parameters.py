"""Controller settings; AVLite persists these outside the plugin directory."""


class PluginSettings:
    # Vehicle values are research estimates inherited from the source controller.
    wheelbase: float = 3.115
    max_steer: float = 0.38
    max_steer_rate: float = 1.5
    dt: float = 0.02
    horizon_steps: int = 20
    weight_lateral_error: float = 2.0
    weight_heading_error: float = 2.0
    weight_steering_state: float = 0.15
    weight_steering: float = 0.30
    weight_steering_rate: float = 2.0
    tube_boundary_margin: float = 0.35
    tube_heading_limit: float = 0.55
    tube_disturbance_lateral: float = 0.002
    tube_disturbance_heading: float = 0.0005
    tube_disturbance_steering: float = 0.0002
    tube_feedback_weight_lateral: float = 0.5
    tube_feedback_weight_heading: float = 1.0
    tube_feedback_weight_steering: float = 0.1
    tube_feedback_weight_input: float = 2.0
    tube_max_ancillary_correction: float = 0.08
    tube_steering_time_constant: float = 0.06
    tube_rpi_iterations: int = 80
    tube_rpi_tolerance: float = 1.0e-8
    tube_minimum_nominal_rate_fraction: float = 0.15
    tube_tighten_steering_rate: bool = False
    osqp_verbose: bool = False
    osqp_polishing: bool = False
    # Enable only when every incoming plan is a complete closed circuit.
    closed_path: bool = False
    # Assumed half-widths when the supplied plan has no signed boundaries.
    default_left_width: float = 4.0
    default_right_width: float = 4.0
    # Small longitudinal helper, independent of the lateral MPC optimization.
    speed_gain: float = 1.0
    max_acceleration: float = 4.0
    max_deceleration: float = 8.0
    max_speed: float = 83.3333333333
