import random
from typing import List, Dict, Any

class FaultEngine:
    def __init__(self):
        self.active_faults: List[Dict[str, Any]] = []

    def inject_fault(self, fault_type: str, severity: float = 1.0):
        """
        Adds a fault to the active list.
        fault_type: 'motor_overcurrent', 'sensor_timeout', 'comms_loss'
        """
        self.active_faults.append({
            'type': fault_type,
            'severity': severity
        })

    def clear_faults(self):
        self.active_faults.clear()

    def apply_faults(self, state: Any):
        """Modifies the gimbal state based on active faults."""
        for fault in self.active_faults:
            if fault['type'] == 'motor_overcurrent':
                # Simulate a jitter/instability in position
                state.physics.pan['pos'] += random.uniform(-1.0, 1.0) * fault['severity']
                state.physics.tilt['pos'] += random.uniform(-1.0, 1.0) * fault['severity']
            
            elif fault['type'] == 'sensor_timeout':
                # Simulate sensor data freezing/stalling
                # In a real simulator, this might stop updating gps_lat/lon
                # We simulate this by injecting zero-delta updates or holding old values
                # For this simple implementation, we'll just set a flag that the state can use
                state.is_faulty = True 
            
            elif fault['type'] == 'comms_loss':
                # Simulate data corruption or loss
                # In the context of a server, this might mean the server stops reading
                # For the state, we can simulate it by clearing the initialized flag
                state.initialized = False
