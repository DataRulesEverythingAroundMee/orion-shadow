import struct
import logging
from typing import List, Dict, Any

logger = logging.getLogger(__name__)

class FaultEngine:
    def __init__(self):
        self.active_faults: List[Dict[str, Any]] = []

    def inject_fault(self, fault_type: str, severity: float = 1.0):
        """
        Adds a fault to the active list.
        fault_type: 'motor_overcurrent', 'sensor_timeout', 'comms_loss'
        """
        mapping = {'motor_overcurrent', 'sensor_timeout', 'comms_loss'}
        if fault_type not in mapping:
            logger.warning("Injecting unrecognized fault type '%s'", fault_type)
        else:
            logger.info("Fault injected: '%s' (severity=%.2f)", fault_type, severity)

        self.active_faults.append({
            'type': fault_type,
            'severity': severity
        })

    def clear_faults(self):
        count = len(self.active_faults)
        self.active_faults.clear()
        logger.info("Cleared all active faults (%d fault(s) removed)", count)

    def get_active_fault_ids(self) -> List[int]:
        """
        Returns a list of numeric fault IDs for transmission.
        """
        mapping = {
            'motor_overcurrent': 1,
            'sensor_timeout': 2,
            'comms_loss': 3
        }
        ids = [mapping[f['type']] for f in self.active_faults if f['type'] in mapping]
        if logger.isEnabledFor(logging.DEBUG):
            logger.debug("Active fault IDs resolved: %s", ids)
        return ids

    def apply_faults(self, state: Any):
        """Modifies the gimbal state based on active faults."""
        if self.active_faults and logger.isEnabledFor(logging.DEBUG):
            logger.debug("Applying %d active faults to gimbal state", len(self.active_faults))

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
