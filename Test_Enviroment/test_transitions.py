import numpy as np
import sys
import os

# Mock the environment to import modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from shared_state import shared_state, ContactState
import perception

def test_transitions():
    print("Testing Robust State Transitions...")
    
    # 1. Start in NO_CONTACT
    shared_state.reset()
    perception.reset_perception()
    shared_state.left_foot_position = np.array([0, 0, 1.0]) # High height
    state, _ = perception.update_perception()
    print(f"Initial State: {state.name} (Expected: NO_CONTACT)")
    
    # 2. Descend to TOUCH_EXPECTED
    shared_state.left_foot_position = np.array([0, 0, 0.01]) # Low height
    state, _ = perception.update_perception()
    print(f"Descended State: {state.name} (Expected: TOUCH_EXPECTED)")
    
    # 3. First Tick: Force > Threshold -> CONTACT_TENTATIVE
    shared_state.left_foot_force = 20.0
    shared_state.left_contact_ticks = 1 # Simulation would do this
    state, _ = perception.update_perception()
    print(f"Tick 1 State: {state.name} (Expected: CONTACT_TENTATIVE)")
    
    # 4. Second Tick: Force stays -> Still CONTACT_TENTATIVE
    shared_state.left_contact_ticks = 2
    state, _ = perception.update_perception()
    print(f"Tick 2 State: {state.name} (Expected: CONTACT_TENTATIVE)")
    
    # 5. Third Tick: Force stays, but NOT FLAT -> Still CONTACT_TENTATIVE
    shared_state.left_contact_ticks = 3
    shared_state.left_foot_flat = False
    state, _ = perception.update_perception()
    print(f"Tick 3 State (Not Flat): {state.name} (Expected: CONTACT_TENTATIVE)")
    
    # 6. Third Tick + FLAT -> CONTACT_CONFIRMED
    shared_state.left_foot_flat = True
    state, _ = perception.update_perception()
    print(f"Tick 3 State (Flat): {state.name} (Expected: CONTACT_CONFIRMED)")
    
    # 7. Force drops -> Reset to TOUCH_EXPECTED or NO_CONTACT
    shared_state.left_contact_ticks = 0
    shared_state.left_foot_force = 1.0 # Below noise
    state, _ = perception.update_perception()
    print(f"Force Drop State: {state.name} (Expected: TOUCH_EXPECTED because h is low)")

if __name__ == "__main__":
    test_transitions()


import math
import pytest


# ── Task 1 tests ──────────────────────────────────────────────────────────── #

def test_foot_pitch_fields_exist_after_init():
    """shared_state has left_foot_pitch and right_foot_pitch initialised to 0."""
    from shared_state import Siclo1State
    s = Siclo1State()
    assert hasattr(s, 'left_foot_pitch')
    assert hasattr(s, 'right_foot_pitch')
    assert s.left_foot_pitch == 0.0
    assert s.right_foot_pitch == 0.0


def test_foot_pitch_fields_reset_to_zero():
    """reset() brings left_foot_pitch and right_foot_pitch back to 0."""
    shared_state.left_foot_pitch  = 0.5
    shared_state.right_foot_pitch = -0.3
    shared_state.reset()
    assert shared_state.left_foot_pitch  == 0.0
    assert shared_state.right_foot_pitch == 0.0


# ── Task 2 tests ──────────────────────────────────────────────────────────── #

class TestComputeFootFlat:
    """Unit tests for the _compute_foot_flat pure helper in HeartBeat.py."""

    def test_single_contact_low_pitch_is_flat(self):
        """Single contact point + pitch 4° (< 7°) → flat confirmed."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0], math.radians(4.0)) is True

    def test_single_contact_high_pitch_not_flat(self):
        """Single contact point + pitch 15° (> 7°) → tiptoe, not confirmed."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0], math.radians(15.0)) is False

    def test_single_contact_exactly_threshold_not_flat(self):
        """7° exactly is rejected — gate is strictly less than."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0], math.radians(7.0)) is False

    def test_single_contact_negative_pitch_accepted(self):
        """Negative pitch (heel slightly raised on opposite side) within 7° → flat."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0], math.radians(-4.0)) is True

    def test_multi_point_wide_spread_is_flat(self):
        """Multiple contact points with 3 cm spread → flat regardless of pitch."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0, 0.03], math.radians(45.0)) is True

    def test_multi_point_narrow_spread_not_flat(self):
        """Multiple contact points with 0.5 cm spread → NOT flat (below 1 cm gate)."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([0.0, 0.005], math.radians(0.0)) is False

    def test_no_contact_points_not_flat(self):
        """Empty contact list → not flat."""
        from HeartBeat import _compute_foot_flat
        assert _compute_foot_flat([], math.radians(0.0)) is False

    def test_tick_gate_still_required(self):
        """Foot flat=True but ticks=2 → state stays CONTACT_TENTATIVE (ticks gate untouched)."""
        from shared_state import shared_state, ContactState
        shared_state.reset()
        perception.reset_perception()
        shared_state.left_foot_position = np.array([0.0, 0.0, 0.01])
        shared_state.left_contact_ticks = 2
        shared_state.left_foot_flat = True
        state, _ = perception.update_perception()
        assert state == ContactState.CONTACT_TENTATIVE
