import math
from typing import Dict

class PhysicsEngine:
    """Simulates gimbal dynamics: inertia, velocity, and acceleration."""
    def __init__(self, dt: float = 0.1):
        self.dt = dt
        # Current state: [pos, vel, acc]
        self.pan = {"pos": 0.0, "vel": 0.0, "acc": 0.0}
        self.tilt = {"pos": 0.0, "vel": 0.0, "acc": 0.0}
        
        # Physical constants (approximating a heavy gimbal)
        self.max_vel = 60.0  # deg/s
        self.max_acc = 100.0 # deg/s^2
        self.damping = 0.95  # simplistic friction/damping

    def step(self, target_pan: float, target_tilt: float):
        """Integrates physics one timestep forward."""
        self._update_axis(self.pan, target_pan)
        self._update_axis(self.tilt, target_tilt)

    def _update_axis(self, axis: Dict[str, float], target: float):
        # Error to target
        error = target - axis["pos"]
        
        # Simple Proportional control for acceleration
        desired_acc = error * 10.0 
        
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
        
        # Apply damping
        axis["vel"] *= self.damping
