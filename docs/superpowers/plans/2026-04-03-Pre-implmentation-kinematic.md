Project Siclo1 — Session Handoff Summary

  Date: 2026-04-03 | Week: 3 | Status: Pre-implementation (kinematics.py not yet written)

  ---
  I. Logic Gate Solutions (Approved)

  Q1 — Kinematic Limits (Workspace Clamping)
  Two-link planar IK has a reachable annulus bounded by:
  - R_min = |L_thigh − L_shank| + SINGULARITY_BUFFER (avoid straight-line collapse)
  - R_max = L_thigh + L_shank − SINGULARITY_BUFFER (avoid full extension lock)

  Any foot target outside this annulus is clamped radially to the boundary before IK is solved.
  Buffer = 0.005 m.

  Q2 — Swing Trajectory (Cycloidal)
  Foot trajectory uses a cycloidal profile to guarantee zero velocity at liftoff and touchdown:

  phi ∈ [0, 1]  (normalized phase)
  x(phi) = x_start + (x_end − x_start) × [phi − sin(2π·phi)/(2π)]
  z(phi) = H × [1 − cos(2π·phi)] / 2

  H = swing clearance height (e.g. 0.04 m). Velocity = 0 at phi=0 and phi=1 by construction.

  Q3 — Angular Momentum Compensation (Torso Pitch)
  During swing, the swing leg deviates from neutral → angular momentum imbalance. Feedforward torso
  pitch correction:

  Δθ_torso = −(m_leg / m_total) × Δθ_hip_swing

  Applied as a feedforward offset on the torso pitch joint. LIPM capture-point feedback (already in
  active_balance.py) handles residual error.

  Q4 — Unit Test Snippet
  def assert_ik_within_urdf_limits(hip_f, knee, ankle, side):
      limits = URDF_JOINT_LIMITS
      hip_name  = 'Left_Hip_Forwards'  if side == 'left' else 'Right_Hip_Fowards'
      knee_name = 'Left_Knee'          if side == 'left' else 'Right_Knee'
      ankle_name= 'Left_Ankle'         if side == 'left' else 'Right_Ankle'
      assert limits[hip_name]['lower']  <= hip_f <= limits[hip_name]['upper']
      assert limits[knee_name]['lower'] <= knee  <= limits[knee_name]['upper']
      assert limits[ankle_name]['lower']<= ankle <= limits[ankle_name]['upper']

  ---
  II. URDF Joint Name Map (Exact — Typos Preserved)

  ┌────────────────┬──────────────────────┬─────────────────────────┬──────────────────────────┐
  │  Logical Role  │   URDF Joint Name    │          Axis           │       Limits (rad)       │
  ├────────────────┼──────────────────────┼─────────────────────────┼──────────────────────────┤
  │ Left Hip Yaw   │ Left_Hip_Twist       │ [0,0,1]                 │ ±0.349066                │
  ├────────────────┼──────────────────────┼─────────────────────────┼──────────────────────────┤
  │ Left Hip       │ Left_Hip_Inwards     │ [-1,0,0] (Y)            │ ±0.698132                │
  │ Abduction      │                      │                         │                          │
  ├────────────────┼──────────────────────┼─────────────────────────┼──────────────────────────┤
  │ Left Hip Pitch │ Left_Hip_Forwards    │ [-1,0,0]                │ ±1.570796                │
  ├────────────────┼──────────────────────┼─────────────────────────┼──────────────────────────┤
  │ Left Knee      │ Left_Knee            │ [-1,0,0]                │ ±1.570796                │
  ├────────────────┼──────────────────────┼─────────────────────────┼──────────────────────────┤
  │ Left Ankle     │ Left_Ankle           │ [0, 0.005365,           │ ±0.349066                │
  │                │                      │ -0.999986]              │                          │
  ├────────────────┼──────────────────────┼─────────────────────────┼──────────────────────────┤
  │ Right Hip Yaw  │ Right_Hip_Twist      │ [0,0,-1]                │ −0.034907 to +0.349066   │
  │                │                      │                         │ (asymmetric)             │
  ├────────────────┼──────────────────────┼─────────────────────────┼──────────────────────────┤
  │ Right Hip      │ Right_Hip_Inwards    │ [0,-1,0]                │ ±0.698132                │
  │ Abduction      │                      │                         │                          │
  ├────────────────┼──────────────────────┼─────────────────────────┼──────────────────────────┤
  │ Right Hip      │ Right_Hip_Fowards ←  │ [+1,0,0]                │ ±1.570796                │
  │ Pitch          │ typo                 │                         │                          │
  ├────────────────┼──────────────────────┼─────────────────────────┼──────────────────────────┤
  │ Right Knee     │ Right_Knee           │ [+1,0,0]                │ ±1.570796                │
  ├────────────────┼──────────────────────┼─────────────────────────┼──────────────────────────┤
  │ Right Ankle    │ Right_Ankle          │ [0,-0.005365,-0.999986] │ ±0.349066                │
  └────────────────┴──────────────────────┴─────────────────────────┴──────────────────────────┘

  Axis-sign rule (critical for IK):
  - Left hip/knee: axis = −X → negate joint angle before applying sin/cos
  - Right hip/knee: axis = +X → no negation

  ---
  III. URDF Segment Lengths (Verified from <origin> tags)

  Euclidean distance ‖xyz‖ of each joint origin:

  ┌─────────────┬──────────────────────────────────┬─────────────────────────────┬────────────┐
  │   Segment   │              Joint               │             xyz             │ Length (m) │
  ├─────────────┼──────────────────────────────────┼─────────────────────────────┼────────────┤
  │ Left Thigh  │ Left_Knee in Left_Upper_Leg_1    │ 0.031, 0.000969, -0.052133  │ 0.060661   │
  ├─────────────┼──────────────────────────────────┼─────────────────────────────┼────────────┤
  │ Left Shank  │ Left_Ankle in Left_Lower_Leg_1   │ 0.1, 0.024043, -0.679218    │ 0.686961   │
  ├─────────────┼──────────────────────────────────┼─────────────────────────────┼────────────┤
  │ Right Thigh │ Right_Knee in Right_Upper_Leg_1  │ -0.106, -0.013692, -0.00859 │ 0.107225   │
  ├─────────────┼──────────────────────────────────┼─────────────────────────────┼────────────┤
  │ Right Shank │ Right_Ankle in Right_Lower_Leg_1 │ -0.025, 0.010133, -0.758742 │ 0.759221   │
  └─────────────┴──────────────────────────────────┴─────────────────────────────┴────────────┘

  ---
  IV. Symmetry Audit — PENDING ⚠️ 
                                 
  Discrepancy confirmed:
                                         
  ┌──────────────┬────────────┬────────────┬─────────────┐
  │              │  Left Leg  │ Right Leg  │    Delta    │
  ├──────────────┼────────────┼────────────┼─────────────┤
  │ Thigh length │ 0.060661 m │ 0.107225 m │ +0.046564 m │                                          
  ├──────────────┼────────────┼────────────┼─────────────┤                                          
  │ Shank length │ 0.686961 m │ 0.759221 m │ +0.072260 m │                                          
  ├──────────────┼────────────┼────────────┼─────────────┤                                          
  │ Total R_max  │ 0.747622 m │ 0.866446 m │ +0.118824 m │
  └──────────────┴────────────┴────────────┴─────────────┘
                                                                                                    
  The right leg is 11.9 cm longer than the left. This is not a modeling choice — it is an asymmetric
   URDF export artifact from Fusion 360. The Symmetry Correction Audit was interrupted before       
  corrective XML was generated.                                                                     
                                         
  Audit is PENDING. The corrected <origin> values and Fusion 360 Adjustment Log have not been       
  produced yet. kinematics.py must NOT be written until the audit is resolved and symmetric segment
  lengths are confirmed.                                                                            
                                         
  ---                                                                                               
  V. Immediate Next Action                              
                                                                                                    
  BLOCKED: kinematics.py implementation  
  REASON:  Symmetry Audit incomplete — R_max_Left ≠ R_max_Right (delta = 0.1188 m)                  
  COMMAND: Resume Symmetry Audit → produce corrected Right_ joint XML → then GO on kinematics.py
                                                                                                    