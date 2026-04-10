"""
================================================================================
PROJECT SICLO1 - RECOVERY MODULE
================================================================================

Monitors system state and triggers recovery actions.
Responds to instability and contact failures.

INPUTS (from shared_state):
- stability_status (from stability.py)
- is_unstable
- left_foot_slip_detected, right_foot_slip_detected (from perception.py)
- left_foot_contact_state, right_foot_contact_state
- step duration

OUTPUTS (to shared_state):
- recovery_action (NONE/ABORT_HOLD/REPOSITION/EMERGENCY_STOP)
- recovery_active
- recovery_reason

Author: Siclo1 Project Team
Date: February 2026

================================================================================
"""

import numpy as np
from typing import Tuple, List
from dataclasses import dataclass

# Import shared state
from shared_state import (
    shared_state,
    ContactState,
    MissionState,
    StabilityStatus,
    RecoveryAction,
    RecoveryConfig,
)


# ============================================================================
# RECOVERY EVENT TRACKING
# ============================================================================

@dataclass
class RecoveryEvent:
    """Record of a recovery event"""
    timestamp: float
    trigger: str
    action: RecoveryAction
    success: bool
    details: str = ""


# ============================================================================
# RECOVERY CONTROLLER
# ============================================================================

class RecoveryController:
    """
    Monitors shared_state and triggers recovery actions.
    
    Monitors:
    - Stability (from stability.py)
    - Contact (from perception.py)
    - Timeouts
    
    Actions:
    - ABORT_HOLD: Stop motion, hold position
    - REPOSITION: Retract and try again
    - EMERGENCY_STOP: Full system shutdown
    """
    
    def __init__(self, config: RecoveryConfig = None):
        """
        Initialize recovery controller.
        
        Args:
            config: Recovery configuration (uses default if None)
        """
        self.config = config if config is not None else RecoveryConfig()
        
        # Event history
        self.events: List[RecoveryEvent] = []
        
        # Recovery attempt tracking
        self.current_step_attempts = 0
        
        # Previous state tracking
        self.prev_left_contact = ContactState.NO_CONTACT
        self.prev_right_contact = ContactState.NO_CONTACT
    
    def evaluate(self) -> Tuple[RecoveryAction, str]:
        """
        Evaluate if recovery is needed.
        
        Checks (in priority order):
        1. Emergency: Unstable + timeout
        2. Contact lost: Confirmed -> Lost
        3. Timeout: No contact after threshold
        4. Slip detected: Foot slipping
        5. Marginal stability timeout
        
        Reads from shared_state:
        - stability_status, is_unstable
        - left/right_foot_contact_state
        - left/right_foot_slip_detected
        - step duration
        
        Returns:
            (action, reason)
        """
        # Reset step timer every cycle in IDLE so timeout checks (P1, P3, P5) never
        # fire while standing still. Slip (P4) and contact-loss (P2) still evaluate.
        # Side effect: when IDLE→WALK, timer starts from 0 — watchdog begins clean.
        if shared_state.mission_state == MissionState.IDLE:
            shared_state.current_step_start_time = shared_state.sim_time

        # Get current state
        step_duration = shared_state.get_step_duration()
        
        # ====================================================================
        # PRIORITY 1: EMERGENCY - Unstable + Timeout
        # ====================================================================
        if (shared_state.is_unstable and 
            step_duration > self.config.timeout_threshold):
            return RecoveryAction.EMERGENCY_STOP, "Unstable timeout - critical failure"
        
        # ====================================================================
        # PRIORITY 2: CONTACT LOST - Unexpected contact loss
        # ====================================================================
        # Left foot contact lost
        if (self.prev_left_contact == ContactState.CONTACT_CONFIRMED and
            shared_state.left_foot_contact_state in [ContactState.NO_CONTACT, 
                                                     ContactState.TOUCH_EXPECTED]):
            return RecoveryAction.ABORT_HOLD, "Left foot contact lost unexpectedly"
        
        # Right foot contact lost
        if (self.prev_right_contact == ContactState.CONTACT_CONFIRMED and
            shared_state.right_foot_contact_state in [ContactState.NO_CONTACT,
                                                      ContactState.TOUCH_EXPECTED]):
            return RecoveryAction.ABORT_HOLD, "Right foot contact lost unexpectedly"
        
        # ====================================================================
        # PRIORITY 3: TIMEOUT - No contact after threshold
        # ====================================================================
        both_not_confirmed = (
            shared_state.left_foot_contact_state != ContactState.CONTACT_CONFIRMED and
            shared_state.right_foot_contact_state != ContactState.CONTACT_CONFIRMED
        )
        
        if (both_not_confirmed and
                step_duration > self.config.timeout_threshold and
                shared_state.mission_state != MissionState.IDLE):
            # Check if we've tried too many times
            if self.current_step_attempts >= self.config.max_recovery_attempts:
                return RecoveryAction.EMERGENCY_STOP, "Max recovery attempts exceeded"
            else:
                return RecoveryAction.REPOSITION, "Timeout without contact - reposition"
        
        # ====================================================================
        # PRIORITY 4: SLIP DETECTED
        # ====================================================================
        if shared_state.any_foot_slipping():
            foot = "left" if shared_state.left_foot_slip_detected else "right"
            return RecoveryAction.REPOSITION, f"{foot.capitalize()} foot slip detected"
        
        # ====================================================================
        # PRIORITY 5: MARGINAL STABILITY TIMEOUT
        # ====================================================================
        if (shared_state.stability_status == StabilityStatus.MARGINAL and
            step_duration > self.config.marginal_timeout):
            return RecoveryAction.ABORT_HOLD, "Marginal stability timeout"
        
        # ====================================================================
        # NO RECOVERY NEEDED
        # ====================================================================
        return RecoveryAction.NONE, ""
    
    def execute(self, action: RecoveryAction, reason: str) -> bool:
        """
        Execute recovery action.
        
        Args:
            action: Recovery action to execute
            reason: Reason for recovery
        
        Writes to shared_state:
        - recovery_action
        - recovery_active
        - recovery_reason
        - system_status (if degraded/emergency)
        
        Returns:
            success (bool)
        """
        # Log event
        event = RecoveryEvent(
            timestamp=shared_state.sim_time,
            trigger=reason,
            action=action,
            success=False,  # Updated below
            details=""
        )
        
        # Update shared state
        shared_state.set_recovery_action(action, reason)
        
        # Execute action
        if action == RecoveryAction.EMERGENCY_STOP:
            # Critical failure - stop everything
            shared_state.emergency_stop_triggered = True
            shared_state.freeze_robot = True
            event.success = False
            event.details = "System halted"
            
        elif action == RecoveryAction.ABORT_HOLD:
            # Hold current position
            self.current_step_attempts += 1
            event.success = True
            event.details = f"Holding position (attempt {self.current_step_attempts})"
            
        elif action == RecoveryAction.REPOSITION:
            # Retract and try again
            self.current_step_attempts += 1
            event.success = True
            event.details = f"Repositioning (attempt {self.current_step_attempts})"
            
        # Store event
        self.events.append(event)
        
        return event.success
    
    def update(self):
        """
        Main update function called by main.py.
        
        Process:
        1. Evaluate if recovery needed
        2. Execute recovery if needed
        3. Update previous state
        """
        # Evaluate recovery
        action, reason = self.evaluate()
        
        # Execute if needed
        if action != RecoveryAction.NONE:
            self.execute(action, reason)
        else:
            # Clear recovery flags if no longer needed
            if shared_state.recovery_active:
                shared_state.set_recovery_action(RecoveryAction.NONE, "")
        
        # Update previous state for next cycle
        self.prev_left_contact = shared_state.left_foot_contact_state
        self.prev_right_contact = shared_state.right_foot_contact_state
    
    def reset_step(self):
        """
        Reset recovery tracking for new step.
        Called when step successfully completes.
        """
        self.current_step_attempts = 0
        shared_state.current_step_start_time = shared_state.sim_time
    
    def get_statistics(self) -> dict:
        """Get recovery statistics"""
        total_events = len(self.events)
        successful = sum(1 for e in self.events if e.success)
        
        action_counts = {
            'ABORT_HOLD': 0,
            'REPOSITION': 0,
            'EMERGENCY_STOP': 0,
        }
        
        for event in self.events:
            if event.action.name in action_counts:
                action_counts[event.action.name] += 1
        
        return {
            'total_events': total_events,
            'successful_recoveries': successful,
            'failed_recoveries': total_events - successful,
            'abort_hold_count': action_counts['ABORT_HOLD'],
            'reposition_count': action_counts['REPOSITION'],
            'emergency_stop_count': action_counts['EMERGENCY_STOP'],
        }
    
    def print_event_log(self, last_n: int = 10):
        """Print recent recovery events"""
        print(f"\n{'='*70}")
        print(f"RECOVERY EVENT LOG (Last {last_n})")
        print(f"{'='*70}")
        print(f"{'Time':<10} {'Action':<18} {'Success':<8} {'Trigger':<30}")
        print(f"{'-'*70}")
        
        for event in self.events[-last_n:]:
            success_str = '✓' if event.success else '✗'
            print(f"{event.timestamp:<10.3f} {event.action.name:<18} "
                  f"{success_str:<8} {event.trigger:<30}")
        
        print(f"{'='*70}\n")


# ============================================================================
# GLOBAL INSTANCE
# ============================================================================

# Create global recovery controller
recovery_controller = RecoveryController()


# ============================================================================
# PUBLIC API
# ============================================================================

def update_recovery():
    """
    Public function called by main.py.
    
    Monitors shared_state and triggers recovery as needed.
    """
    recovery_controller.update()


def reset_step():
    """
    Reset recovery tracking for new step.
    Call when step completes successfully.
    """
    recovery_controller.reset_step()


def set_recovery_config(config: RecoveryConfig):
    """Update recovery configuration"""
    recovery_controller.config = config


def get_recovery_statistics() -> dict:
    """Get recovery performance statistics"""
    return recovery_controller.get_statistics()


def print_recovery_log(last_n: int = 10):
    """Print recent recovery events"""
    recovery_controller.print_event_log(last_n)


# ============================================================================
# TESTING
# ============================================================================

if __name__ == "__main__":
    """Test recovery module"""
    print("="*70)
    print("RECOVERY MODULE TEST")
    print("="*70)
    
    # Initialize shared state
    shared_state.reset()
    shared_state.current_step_start_time = 0.0
    
    # Test 1: Timeout scenario
    print("\nTest 1: Timeout without contact")
    shared_state.sim_time = 4.0  # 4 seconds elapsed
    shared_state.left_foot_contact_state = ContactState.TOUCH_EXPECTED
    shared_state.right_foot_contact_state = ContactState.TOUCH_EXPECTED
    
    update_recovery()
    
    print(f"  Action: {shared_state.recovery_action.name}")
    print(f"  Reason: {shared_state.recovery_reason}")
    print(f"  Expected: REPOSITION")
    
    # Test 2: Contact loss scenario
    print("\nTest 2: Contact loss")
    shared_state.sim_time = 0.0
    shared_state.current_step_start_time = 0.0
    recovery_controller.prev_left_contact = ContactState.CONTACT_CONFIRMED
    shared_state.left_foot_contact_state = ContactState.NO_CONTACT
    
    update_recovery()
    
    print(f"  Action: {shared_state.recovery_action.name}")
    print(f"  Reason: {shared_state.recovery_reason}")
    print(f"  Expected: ABORT_HOLD")
    
    # Test 3: Unstable + timeout (emergency)
    print("\nTest 3: Emergency (unstable + timeout)")
    shared_state.sim_time = 5.0
    shared_state.current_step_start_time = 0.0
    shared_state.set_stability_status(StabilityStatus.UNSTABLE, margin=-0.05)
    recovery_controller.prev_left_contact = ContactState.NO_CONTACT
    
    update_recovery()
    
    print(f"  Action: {shared_state.recovery_action.name}")
    print(f"  Reason: {shared_state.recovery_reason}")
    print(f"  Expected: EMERGENCY_STOP")
    print(f"  Emergency triggered: {shared_state.emergency_stop_triggered}")
    
    # Test 4: Slip detection
    print("\nTest 4: Slip detection")
    shared_state.reset()
    shared_state.set_slip_detection('left', True)
    
    update_recovery()
    
    print(f"  Action: {shared_state.recovery_action.name}")
    print(f"  Reason: {shared_state.recovery_reason}")
    print(f"  Expected: REPOSITION")
    
    # Print statistics
    print("\nRecovery Statistics:")
    stats = get_recovery_statistics()
    for key, value in stats.items():
        print(f"  {key}: {value}")
    
    # Print event log
    print_recovery_log(last_n=10)
    
    print("="*70)
    print("✓ Recovery module test complete")
    print("="*70)
