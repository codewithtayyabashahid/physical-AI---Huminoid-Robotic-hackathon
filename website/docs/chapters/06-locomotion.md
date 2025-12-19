---
id: 06-locomotion
title: Humanoid Locomotion
sidebar_position: 6
---

# Humanoid Locomotion

Bipedal walking is one of the most challenging problems in robotics, requiring coordination of many degrees of freedom while maintaining balance on two feet.

## Why Humanoid Robots?

Advantages of human-like morphology:
- Navigate human-built environments (stairs, doors)
- Use human tools without modification
- Natural interaction with people
- Leverage human motion data for learning

## The Balance Problem

### Center of Mass (CoM)

Maintaining stability requires controlling CoM:
- Static stability: CoM over support polygon
- Dynamic stability: Zero Moment Point (ZMP)
- Capture point: where robot must step to stop

### Zero Moment Point (ZMP)

Point where net ground reaction moment is zero:
- Must lie within support polygon
- ZMP-based controllers ensure stability
- Conservative but reliable approach
```python
def compute_zmp(com_pos, com_acc, g=9.81):
    """Compute ZMP position from CoM dynamics"""
    zmp_x = com_pos[0] - (com_pos[2] / g) * com_acc[0]
    zmp_y = com_pos[1] - (com_pos[2] / g) * com_acc[1]
    return [zmp_x, zmp_y]
```

## Walking Gait Generation

### Trajectory Optimization

Generate walking motion:
- Objective: minimize energy, maximize stability
- Constraints: ZMP, joint limits, no slip
- Contact sequence: single support, double support
- Preview control for future steps

### Model Predictive Control (MPC)

Real-time walking control:
- Simplified model (linear inverted pendulum)
- Solve optimization every timestep
- Handles disturbances online
- Used by Boston Dynamics Atlas

### Central Pattern Generators (CPG)

Biologically-inspired rhythmic controllers:
- Coupled oscillators generate periodic motion
- Parameters modulate gait characteristics
- Smooth transitions between gaits
- Robust to perturbations

## Modern Approaches

### Learning-Based Locomotion

Deep RL for walking:
- Train in simulation (Isaac Gym, MuJoCo)
- Reward shaping: forward velocity, energy, stability
- Domain randomization for robustness
- Transfer to real hardware

### Whole-Body Control

Unified controller for all joints:
- Quadratic programming (QP) formulation
- Hierarchical tasks: balance > tracking > regularization
- Contact forces as decision variables
- Real-time on humanoid robots
```python
# Simplified whole-body control structure
def whole_body_control(robot_state, desired_motion):
    # Decision variables: joint accelerations and contact forces
    qdd = cp.Variable(n_joints)
    F_contact = cp.Variable(n_contacts * 3)
    
    # Dynamics constraint
    M @ qdd + h == S.T @ tau + J_c.T @ F_contact
    
    # Tracking objective
    cost = cp.sum_squares(qdd - qdd_desired)
    
    # Friction cone constraints
    constraints = [friction_cone(F_contact)]
    
    problem = cp.Problem(cp.Minimize(cost), constraints)
    problem.solve()
    return qdd.value
```

## Terrain Adaptation

Walking on complex terrain:

### Perception Integration

- Height maps from depth sensors
- Footstep placement planning
- Step adjustment based on ground contact

### Compliance Control

- Impedance control at ankles
- Absorb impact forces
- Adapt to uneven surfaces

## Robust Locomotion

Handling disturbances:

### Push Recovery

Strategies for regaining balance:
- Ankle strategy: small perturbations
- Hip strategy: moderate disturbances
- Step strategy: large pushes
- Learned recovery policies

### Fall Detection and Protection

- IMU-based fall detection
- Emergency stop routines
- Compliant joints to reduce impact
- Safety roll behaviors

## State-of-the-Art Humanoids

### Boston Dynamics Atlas

- Hydraulic actuation for high power
- Whole-body control with QP
- Parkour and backflips
- Advanced perception for autonomy

### Tesla Optimus

- Electric actuators for efficiency
- Vision-based perception (no LiDAR)
- Learning-based control
- Designed for manufacturing scale

### Agility Robotics Digit

- Torso-mounted battery pack
- Compliant legs for robustness
- Manipulation capabilities
- Commercial deployment for logistics

### Sanctuary AI Phoenix

- Bimanual manipulation focus
- Human-like hands (20 DOF each)
- Teleoperation for data collection
- Carbon fiber construction

## Energy Efficiency

Efficient walking is crucial:
- Passive dynamics: exploit natural swing
- Energy-optimal gaits
- Regenerative braking
- Lightweight design

## Simulation and Testing

Before hardware deployment:

### Physics Simulators

- Accurate contact modeling
- Realistic friction and compliance
- Parallel environments for learning
- Stress testing in virtual scenarios

### Hardware-in-the-Loop

- Test controllers on actual actuators
- Validate sensor processing
- Identify real-world issues early

## Future Directions

- Learned locomotion primitives
- Adaptation to user preferences
- Running and jumping on uneven terrain
- Human-level agility and efficiency

## Summary

Humanoid locomotion combines classical control theory with modern machine learning. Challenges remain in robustness, efficiency, and versatility, but rapid progress is making capable humanoid robots a reality. Next, we explore manipulation—how humanoids interact with objects.