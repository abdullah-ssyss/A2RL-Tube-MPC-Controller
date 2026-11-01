"""Convert an externally supplied AVLite path into a lateral MPC reference."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PathReference:
    lateral_error: float
    heading: float
    curvature: float
    preview: tuple[float, ...]
    speed: float
    lower: float
    upper: float


def sample_reference(path, velocity, left, right, ego, setting, dt) -> PathReference:
    """Project onto path segments and preview by physical arc length.

    Signed plan boundaries are measured relative to this supplied path. Local
    plans without boundaries use the explicit assumed widths from settings.
    A scalar corridor uses the narrowest bounds over the prediction horizon.
    """
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        raise ValueError("The reference path needs at least three XY points")
    if not np.isfinite(points).all():
        raise ValueError("Non-finite reference path")
    count = len(points)

    def values(data, default):
        array = np.asarray(data, dtype=float)
        if array.size == 0:
            return np.full(count, default)
        if array.shape != (count,) or not np.isfinite(array).all():
            raise ValueError("Plan speeds/boundaries must be finite and match path length")
        return array

    speeds = values(velocity, 0.0)
    upper = values(left, setting.default_left_width)
    lower = values(right, -setting.default_right_width)
    if np.any(speeds < 0) or np.any(lower >= upper):
        raise ValueError("Invalid speed profile or lateral corridor")
    closed = setting.closed_path
    if closed and np.linalg.norm(points[-1] - points[0]) < 1.0e-8:
        points, speeds, upper, lower = [a[:-1] for a in (points, speeds, upper, lower)]
    # Drop consecutive duplicates together with their corresponding samples.
    keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1.0e-8]
    points, speeds, upper, lower = [a[keep] for a in (points, speeds, upper, lower)]
    if len(points) < 3:
        raise ValueError("The reference path needs three distinct consecutive points")
    ends = np.roll(points, -1, axis=0) if closed else points[1:]
    starts = points if closed else points[:-1]
    delta = ends - starts
    lengths = np.linalg.norm(delta, axis=1)
    arc = np.r_[0.0, np.cumsum(lengths)]
    xy = np.asarray([ego.x, ego.y], dtype=float)
    fraction = np.clip(np.sum((xy - starts) * delta, axis=1) / lengths**2, 0, 1)
    projections = starts + fraction[:, None] * delta
    segment = int(np.argmin(np.sum((xy - projections)**2, axis=1)))
    direction = delta[segment] / lengths[segment]
    error = xy - projections[segment]
    lateral = float(direction[0] * error[1] - direction[1] * error[0])
    current_s = arc[segment] + fraction[segment] * lengths[segment]
    segment_heading = np.unwrap(np.arctan2(delta[:, 1], delta[:, 0]))
    if closed:
        # Centered tangent at each vertex, with angles unwrapped across the seam.
        prev = np.r_[segment_heading[-1], segment_heading[:-1]]
        turn = np.arctan2(np.sin(segment_heading - prev), np.cos(segment_heading - prev))
        heading = np.unwrap(segment_heading - turn / 2)
        seam_turn = np.arctan2(np.sin(heading[0] - heading[-1]), np.cos(heading[0] - heading[-1]))
        heading = np.r_[heading, heading[-1] + seam_turn]
        speeds, upper, lower = [np.r_[a, a[0]] for a in (speeds, upper, lower)]
    else:
        heading = np.r_[segment_heading[0],
                        (segment_heading[:-1] + segment_heading[1:]) / 2,
                        segment_heading[-1]]
    curvature = np.gradient(heading, arc, edge_order=1)
    preview_s = current_s + np.arange(setting.horizon_steps + 1) * max(ego.velocity, 0.5) * dt
    preview_s = np.mod(preview_s, arc[-1]) if closed else np.minimum(preview_s, arc[-1])
    return PathReference(
        lateral_error=lateral,
        heading=float(np.interp(current_s, arc, heading)),
        curvature=float(np.interp(current_s, arc, curvature)),
        preview=tuple(np.interp(preview_s[1:], arc, curvature)),
        speed=float(np.interp(current_s, arc, speeds)),
        lower=float(np.max(np.interp(preview_s, arc, lower))) + setting.tube_boundary_margin,
        upper=float(np.min(np.interp(preview_s, arc, upper))) - setting.tube_boundary_margin,
    )
