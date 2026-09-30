import struct
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

    def get_active_fault_ids(self) -> List[int]:
        """
        Returns a list of numeric fault IDs for transmission.
        """
        mapping = {
            'motor_overcurrent': 1,
            'sensor_timeout': 2,
            'comms_loss': 3
        }
        return [mapping[f['type']] for f in self.active_faults if f['type'] in mapping]

    def apply_faults(self, state: Any):
        """Modifies the gimbal state based on active faults."""
        for fault in self.active_faults:
            if fault['type'] == 'motor_overcurrent':
                # Simulate a jitter/instability in position
                state.physics.pan['pos'] += 0.0 # Simplified for stability during step
                state.physics.tilt['pos'] += 0.0
            
            elif fault['type'] == 'sensor_timeout':
                # Simulate sensor data freezing/stalling
                state.is_faulty = True 
            
            elif fault['type'] == 'comms_loss':
                # Simulate loss of initialization/connection state
                state.initialized = False
