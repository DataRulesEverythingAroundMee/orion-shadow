import math
import logging
from typing import Dict

logger = logging.getLogger(__name__)

class PhysicsEngine:
    """Simulates gimbal dynamics: inertia, velocity, and acceleration."""
    def __init__(self, dt: float = 0.1, initial_pan: float = 0.0, initial_tilt: float = 0.0):
        self.dt = dt
        # Current state: [pos, vel, acc]
        self.pan = {"pos": float(initial_pan), "vel": 0.0, "acc": 0.0}
        self.tilt = {"pos": float(initial_tilt), "vel": 0.0, "acc": 0.0}
        
        # Physical constants (approximating an agile airborne gimbal)
        self.max_vel = 90.0  # deg/s
        self.max_acc = 200.0 # deg/s^2
        self.damping = 0.95  # simplistic friction/damping

    def step(self, target_pan: float, target_tilt: float, continuous_pan: bool = True,
             tilt_min: float = -80.0, tilt_max: float = 28.0):
        """Integrates physics one timestep forward."""
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                "Physics step: targets (pan=%.2f, tilt=%.2f) | current (pan=%.2f, tilt=%.2f) | vel (%.2f, %.2f)",
                target_pan, target_tilt, self.pan["pos"], self.tilt["pos"], self.pan["vel"], self.tilt["vel"]
            )
        self._update_axis(self.pan, target_pan, continuous=continuous_pan)
        self._update_axis(self.tilt, target_tilt, continuous=False, min_limit=tilt_min, max_limit=tilt_max)

    def _update_axis(self, axis: Dict[str, float], target: float, continuous: bool = False,
                     min_limit: float = None, max_limit: float = None):
        if continuous:
            # Shortest angular distance wrapped to [-180, 180]
            error = (target - axis["pos"] + 180.0) % 360.0 - 180.0
        else:
            if min_limit is not None and max_limit is not None:
                target = max(min(target, max_limit), min_limit)
            error = target - axis["pos"]
        
        # Critically damped PD control for responsive, overshoot-free tracking
        desired_acc = error * 45.0 - axis["vel"] * 13.4
        
        # Clamp acceleration
        desired_acc = max(min(desired_acc, self.max_acc), -self.max_acc)
        
        # Update acceleration
        axis["acc"] = desired_acc
        
        # Update velocity: v = v + a*dt
        axis["vel"] += axis["acc"] * self.dt
        
        # Clamp velocity
        axis["vel"] = max(min(axis["vel"], self.max_vel), -self.max_vel)
        
        # Update position: p = p + v*dt
        axis["pos"] += axis["vel"] * self.dt

        if continuous:
            axis["pos"] = (axis["pos"] + 180.0) % 360.0 - 180.0
        else:
            if min_limit is not None and max_limit is not None:
                if axis["pos"] < min_limit:
                    axis["pos"] = min_limit
                    axis["vel"] = 0.0
                elif axis["pos"] > max_limit:
                    axis["pos"] = max_limit
                    axis["vel"] = 0.0
        
        # Snap to target when settled to prevent lingering fractional error
        if abs(error) < 0.25 and abs(axis["vel"]) < 1.0:
            axis["pos"] = target
            axis["vel"] = 0.0
            axis["acc"] = 0.0

