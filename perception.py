"""
================================================================================
PROJECT SICLO1 - PERCEPTION MODULE
================================================================================

Manages contact state machines for both feet.
Detects contact transitions and slip events.

INPUTS (from shared_state):
- left_foot_position, left_foot_velocity, left_foot_force
- right_foot_position, right_foot_velocity, right_foot_force
- sim_time

OUTPUTS (to shared_state):
- left_foot_contact_state
- right_foot_contact_state
- left_foot_slip_detected
- right_foot_slip_detected

Author: Siclo1 Project Team
Date: February 2026

================================================================================
"""

import numpy as np
from typing import List, Tuple, Dict

# Import shared state
from shared_state import (
    shared_state,
    ContactState,
    ContactConfig,
    ERR_UNSAFE_IMPACT,
    ERR_SLIP_FORCE_DROP,
    ERR_SLIP_LATERAL_VEL,
)


# ============================================================================
# CONTACT STATE MACHINE (PER FOOT)
# ============================================================================

class FootContactFSM:
    """
    Finite State Machine for a single foot.
    
    States:
    - NO_CONTACT: Foot in air
    - TOUCH_EXPECTED: Approaching ground
    - CONTACT_TENTATIVE: Force detected, settling
    - CONTACT_CONFIRMED: Stable contact verified
    
    Features:
    - Temporal filtering (settling time)
    - Hysteresis (different thresholds for engage/release)
    - Emergency detection (unsafe velocity/penetration)
    - Slip detection (sudden force drop or lateral motion)
    """
    
    def __init__(self, foot_name: str, config: ContactConfig):
        """
        Initialize foot FSM.
        
        Args:
            foot_name: 'left' or 'right'
            config: Contact configuration
        """
        self.foot_name = foot_name
        self.config = config
        
        # State
        self.current_state = ContactState.NO_CONTACT
        self.state_entry_time = 0.0
        
        # Force history for filtering
        self.force_history: List[Tuple[float, float]] = []  # (time, force)
        
        # Flags
        self.emergency_detected = False
        self.slip_detected = False
        
        # Transition log
        self.transition_log: List[Tuple[float, ContactState, str]] = []
        
        # Previous position for slip detection
        self.prev_position: np.ndarray = np.zeros(3)
        self.prev_force: float = 0.0
    
    def update(self, height: float, velocity: np.ndarray, force: float, 
              current_time: float) -> ContactState:
        """
        Update state machine.
        
        Args:
            height: Foot height above ground (m)
            velocity: 3D velocity vector (m/s)
            force: Measured normal force (N)
            current_time: Current simulation time (s)
        
        Returns:
            Current contact state
        """
        prev_state = self.current_state
        # Robust 3-tick transition gate
        ticks = shared_state.left_contact_ticks if self.foot_name == 'left' else shared_state.right_contact_ticks
        is_flat = shared_state.left_foot_flat if self.foot_name == 'left' else shared_state.right_foot_flat
        
        if ticks >= 3 and is_flat:
            self.current_state = ContactState.CONTACT_CONFIRMED
        elif ticks >= 1:
            self.current_state = ContactState.CONTACT_TENTATIVE
        elif height < self.config.height_threshold:
            self.current_state = ContactState.TOUCH_EXPECTED
        else:
            self.current_state = ContactState.NO_CONTACT
            
        # Log transitions
        if self.current_state != prev_state:
            reason = f"{prev_state.name} -> {self.current_state.name} (ticks={ticks}, flat={is_flat})"
            self.transition_log.append((current_time, self.current_state, reason))
            shared_state.set_contact_state(self.foot_name, self.current_state)
        
        return self.current_state
    
    def _handle_no_contact(self, height: float, velocity: np.ndarray, 
                          force: float, t: float):
        """Handle NO_CONTACT state"""
        # Approaching ground?
        if height < self.config.height_threshold:
            self._transition_to(ContactState.TOUCH_EXPECTED, t)
        
        # Unexpected contact?
        elif force > self.config.force_threshold_min:
            self._transition_to(ContactState.CONTACT_TENTATIVE, t)
    
    def _handle_touch_expected(self, height: float, velocity: np.ndarray,
                               force: float, t: float):
        """Handle TOUCH_EXPECTED state"""
        # Safety checks
        vertical_velocity = velocity[2]  # Z-component
        penetration = max(0.0, -height)
        
        if (abs(vertical_velocity) > self.config.max_safe_velocity or
            penetration > self.config.max_penetration):
            self.emergency_detected = True
            shared_state.add_error_code(ERR_UNSAFE_IMPACT)
        
        # Force detected?
        if force > self.config.force_threshold_min:
            self._transition_to(ContactState.CONTACT_TENTATIVE, t)
        
        # Moving away from ground?
        elif height > self.config.height_threshold and vertical_velocity > 0:
            self._transition_to(ContactState.NO_CONTACT, t)
    
    def _handle_contact_tentative(self, height: float, velocity: np.ndarray,
                                  force: float, t: float):
        """Handle CONTACT_TENTATIVE state with settling criteria"""
        time_in_state = t - self.state_entry_time
        
        # Force dropped below threshold?
        if force < self.config.force_threshold_release:
            # Contact lost before confirmation
            if height > self.config.height_threshold:
                self._transition_to(ContactState.NO_CONTACT, t)
            else:
                self._transition_to(ContactState.TOUCH_EXPECTED, t)
            return
        
        # Check settling criteria
        settled = self._check_settled()
        
        if settled and time_in_state >= self.config.settling_time:
            # Contact confirmed!
            self._transition_to(ContactState.CONTACT_CONFIRMED, t)
            self.emergency_detected = False  # Clear emergency flag
    
    def _handle_contact_confirmed(self, height: float, velocity: np.ndarray,
                                  force: float, t: float):
        """Handle CONTACT_CONFIRMED state"""
        # Check for contact release (with hysteresis)
        if force < self.config.force_threshold_release:
            if height > self.config.height_threshold:
                self._transition_to(ContactState.NO_CONTACT, t)
            else:
                self._transition_to(ContactState.TOUCH_EXPECTED, t)
            return
        
        # Slip detection
        self._detect_slip(velocity, force, t)
    
    def _check_settled(self) -> bool:
        """
        Check if contact has settled (stable force, low noise).
        
        Returns:
            True if contact appears stable
        """
        if len(self.force_history) < 3:
            return False
        
        # Get recent force measurements
        recent_forces = [f for _, f in self.force_history[-5:]]
        
        # Check 1: Mean force above confirmed threshold
        mean_force = np.mean(recent_forces)
        if mean_force < self.config.force_threshold_confirmed:
            return False
        
        # Check 2: Low variance (stable)
        force_std = np.std(recent_forces)
        if force_std > self.config.force_noise_std * 3:  # 3-sigma rule
            return False
        
        return True
    
    def _detect_slip(self, velocity: np.ndarray, force: float, t: float):
        """
        Detect slip during confirmed contact.
        
        Slip indicators:
        1. Sudden force drop (>50% in one step)
        2. Excessive lateral velocity while in contact
        """
        # Indicator 1: Sudden force drop
        if self.prev_force > 0:
            force_ratio = force / self.prev_force
            if force_ratio < 0.5:  # 50% drop
                self.slip_detected = True
                shared_state.add_error_code(ERR_SLIP_FORCE_DROP)
        
        # Indicator 2: Lateral velocity
        lateral_velocity = np.linalg.norm(velocity[:2])  # XY components
        if lateral_velocity > 0.3:  # 30cm/s lateral motion
            self.slip_detected = True
            shared_state.add_error_code(ERR_SLIP_LATERAL_VEL)
    
    def _transition_to(self, new_state: ContactState, t: float):
        """Transition to new state"""
        self.current_state = new_state
        self.state_entry_time = t
        
        # Clear slip flag on new contact
        if new_state == ContactState.CONTACT_CONFIRMED:
            self.slip_detected = False
    
    def reset(self):
        """Reset state machine"""
        self.current_state = ContactState.NO_CONTACT
        self.state_entry_time = 0.0
        self.force_history.clear()
        self.emergency_detected = False
        self.slip_detected = False


# ============================================================================
# PERCEPTION MANAGER
# ============================================================================

class PerceptionManager:
    """
    Manages contact state machines for both feet.
    """
    
    def __init__(self, config: ContactConfig = None):
        """
        Initialize perception manager.
        
        Args:
            config: Contact configuration (uses default if None)
        """
        self.config = config if config is not None else ContactConfig()
        
        # Create FSMs for both feet
        self.left_fsm = FootContactFSM('left', self.config)
        self.right_fsm = FootContactFSM('right', self.config)
    
    def update_foot(self, foot: str, position: np.ndarray, velocity: np.ndarray,
                   force: float, current_time: float) -> ContactState:
        """
        Update a single foot's state machine.
        
        Args:
            foot: 'left' or 'right'
            position: 3D position
            velocity: 3D velocity
            force: Normal force
            current_time: Simulation time
        
        Returns:
            Current contact state
        """
        # Calculate height (Z-coordinate)
        height = position[2]
        
        # Get appropriate FSM
        fsm = self.left_fsm if foot == 'left' else self.right_fsm
        
        # Update FSM
        state = fsm.update(height, velocity, force, current_time)
        
        # Write to shared state
        shared_state.set_contact_state(foot, state)
        shared_state.set_slip_detection(foot, fsm.slip_detected)
        
        return state
    
    def update(self):
        """
        Main update function called by main.py.
        
        Reads from shared_state:
        - left_foot_position, left_foot_velocity, left_foot_force
        - right_foot_position, right_foot_velocity, right_foot_force
        - sim_time
        
        Writes to shared_state:
        - left_foot_contact_state, right_foot_contact_state
        - left_foot_slip_detected, right_foot_slip_detected
        """
        current_time = shared_state.sim_time
        
        # Update left foot
        left_state = self.update_foot(
            'left',
            shared_state.left_foot_position,
            shared_state.left_foot_velocity,
            shared_state.left_foot_force,
            current_time
        )
        
        # Update right foot
        right_state = self.update_foot(
            'right',
            shared_state.right_foot_position,
            shared_state.right_foot_velocity,
            shared_state.right_foot_force,
            current_time
        )
        
        return left_state, right_state
    
    def reset(self):
        """Reset both FSMs"""
        self.left_fsm.reset()
        self.right_fsm.reset()


# ============================================================================
# GLOBAL INSTANCE
# ============================================================================

# Create global perception manager
perception_manager = PerceptionManager()


# ============================================================================
# PUBLIC API
# ============================================================================

def update_perception():
    """
    Public function called by main.py.
    
    Returns:
        (left_state, right_state)
    """
    return perception_manager.update()


def reset_perception():
    """Reset all contact state machines"""
    perception_manager.reset()


def set_contact_config(config: ContactConfig):
    """Update contact configuration"""
    perception_manager.config = config
    perception_manager.left_fsm.config = config
    perception_manager.right_fsm.config = config


# ============================================================================
# TESTING
# ============================================================================

if __name__ == "__main__":
    """Test perception module"""
    print("="*70)
    print("PERCEPTION MODULE TEST")
    print("="*70)
    
    # Simulate a foot landing sequence
    print("\nSimulating left foot landing...")
    
    dt = 0.001  # 1ms timesteps
    t = 0.0
    
    # Phase 1: Descending (100ms)
    print("\nPhase 1: Descending")
    for i in range(100):
        height = 0.2 - (i * 0.002)  # Descend from 20cm to 0cm
        shared_state.left_foot_position = np.array([0.0, 0.0, height])
        shared_state.left_foot_velocity = np.array([0.0, 0.0, -0.3])
        shared_state.left_foot_force = 0.0
        shared_state.sim_time = t
        
        left_state, _ = update_perception()
        
        if i % 25 == 0:
            print(f"  t={t:.3f}s: h={height:.3f}m, state={left_state.name}")
        
        t += dt
    
    # Phase 2: Contact and settling (50ms)
    print("\nPhase 2: Contact and settling")
    for i in range(50):
        shared_state.left_foot_position = np.array([0.0, 0.0, 0.0])
        shared_state.left_foot_velocity = np.array([0.0, 0.0, 0.0])
        shared_state.left_foot_force = 20.0 + np.random.normal(0, 2.0)
        shared_state.sim_time = t
        
        left_state, _ = update_perception()
        
        if i % 10 == 0:
            print(f"  t={t:.3f}s: force={shared_state.left_foot_force:.1f}N, state={left_state.name}")
        
        t += dt
    
    # Phase 3: Stable contact (100ms)
    print("\nPhase 3: Stable contact")
    for i in range(100):
        shared_state.left_foot_position = np.array([0.0, 0.0, 0.0])
        shared_state.left_foot_velocity = np.array([0.0, 0.0, 0.0])
        shared_state.left_foot_force = 100.0
        shared_state.sim_time = t
        
        left_state, _ = update_perception()
        
        if i == 0 or i == 99:
            print(f"  t={t:.3f}s: state={left_state.name}")
        
        t += dt
    
    # Phase 4: Slip event
    print("\nPhase 4: Simulating slip (sudden force drop)")
    for i in range(10):
        shared_state.left_foot_position = np.array([0.0, 0.0, 0.0])
        shared_state.left_foot_velocity = np.array([0.3, 0.0, 0.0])  # Lateral motion
        shared_state.left_foot_force = 30.0  # Force drop
        shared_state.sim_time = t
        
        left_state, _ = update_perception()
        
        if i == 0:
            print(f"  t={t:.3f}s: slip={shared_state.left_foot_slip_detected}")
        
        t += dt
    
    # Print transition log
    print("\nTransition Log:")
    for time, state, reason in perception_manager.left_fsm.transition_log:
        print(f"  {time:.3f}s: {reason}")
    
    print("\n" + "="*70)
    print("✓ Perception module test complete")
    print("="*70)
    
    # Check final state
    print(f"\nFinal state:")
    print(f"  Left contact: {shared_state.left_foot_contact_state.name}")
    print(f"  Left slip: {shared_state.left_foot_slip_detected}")
    print(f"  Errors logged: {len(shared_state.error_messages)}")
