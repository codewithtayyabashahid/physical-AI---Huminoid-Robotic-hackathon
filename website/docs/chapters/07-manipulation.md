---
id: 07-manipulation
title: Manipulation and Grasping
sidebar_position: 7
---

# Manipulation and Grasping

Dexterous manipulation enables robots to interact with objects in the physical world, from simple pick-and-place to complex assembly and tool use.

## The Grasping Problem

Grasping involves:
- Perceiving object properties
- Planning grasp poses
- Executing contact control
- Maintaining stable grasps

### Grasp Quality Metrics

Evaluating grasp stability:
- **Force closure**: resist arbitrary external forces
- **Form closure**: geometric constraint alone
- **Q1 metric**: worst-case wrench resistance
- **Ferrari-Canny metric**: minimum wrench magnitude

## Parallel Jaw Grippers

Simplest end-effector:

### Advantages
- Simple control (1 DOF)
- Robust for industrial tasks
- High force capability
- Easy to model

### Grasp Planning
- Antipodal grasps: forces through center of mass
- Friction cone constraints
- Detect grasp points from point clouds
```python
def find_antipodal_grasps(point_cloud, normals):
    """Find valid antipodal grasp candidates"""
    grasps = []
    for i, p1 in enumerate(point_cloud):
        for j, p2 in enumerate(point_cloud[i+1:]):
            # Check if normals oppose each other
            if np.dot(normals[i], normals[j]) < -0.9:
                # Check collision-free approach
                if is_collision_free(p1, p2, point_cloud):
                    grasps.append(Grasp(p1, p2))
    return grasps
```

## Dexterous Hands

Multi-fingered hands for complex manipulation:

### Shadow Hand
- 20 actuated degrees of freedom
- Tendon-driven actuation
- Tactile sensors on fingertips
- Human-like dexterity

### Allegro Hand
- 16 DOF (4 fingers × 4 DOF)
- Direct drive motors
- Compact and lightweight
- Research platform for learning

### Design Considerations
- Anthropomorphic vs. task-specific
- Actuation: motors, tendons, pneumatics
- Sensing: force, tactile, proprioception
- Cost and complexity trade-offs

## Grasp Synthesis

Generating candidate grasps:

### Analytical Methods

Force closure analysis:
- Compute grasp matrix G
- Evaluate wrench space
- Check positive-spanning condition

### Data-Driven Approaches

Learning from datasets:
- **GraspNet**: large-scale grasp dataset
- **6-DOF GraspNet**: full pose estimation
- Neural networks predict grasp success
- Transfer learning across object categories

### Sampling-Based Planning

Random grasp sampling:
- Generate candidate hand poses
- Simulate closure and evaluate
- Rank by quality metric
- Execute best grasp

## Vision for Grasping

Perception pipeline:

### Object Detection and Segmentation
- Mask R-CNN for instance segmentation
- 6D pose estimation
- Category-level grasping without pose

### Grasp Affordance Detection
- Directly predict graspable regions
- Dense pixel-wise predictions
- Real-time inference for reactive grasping

### Depth-Based Grasping
- RGB-D input for spatial reasoning
- Point cloud processing
- Occupancy mapping

## Contact-Rich Manipulation

Beyond grasping:

### In-Hand Manipulation

Reorienting object within hand:
- Finger gaiting: sequential finger repositioning
- Rolling and sliding contacts
- Gravity assist strategies
- Learned dexterous manipulation (OpenAI)

### Pushing and Toppling

Non-prehensile manipulation:
- Physics-based predictions
- Push planning for desired motion
- Singulation in clutter

### Tool Use

Using tools to achieve goals:
- Geometric reasoning about tool-task pairing
- Learning tool affordances
- Generalization to novel tools

## Force Control

Regulating contact forces:

### Impedance Control

Mechanical impedance model:
```python
def impedance_control(x, xd, f_ext, K, B):
    """
    Spring-damper system
    x: current position
    xd: desired position
    f_ext: external force
    K: stiffness, B: damping
    """
    f_desired = K @ (xd - x) - B @ dx - f_ext
    return f_desired
```

### Hybrid Position/Force Control

- Position control in free space
- Force control in contact
- Task frame formulation
- Assembly operations

## Assembly

Precise manipulation tasks:

### Peg-in-Hole

Classic assembly problem:
- Tight tolerances (< 0.1mm)
- Search strategies for alignment
- Compliance for error correction
- Force-guided insertion

### Screwing and Threading

Rotational assembly:
- Detect thread start
- Controlled torque application
- Damage prevention

## Learning-Based Manipulation

Modern approaches using ML:

### Reinforcement Learning

Learning manipulation policies:
- Reward shaping: task success, smoothness
- Sim-to-real transfer
- Meta-learning for quick adaptation

### Imitation Learning

Learning from demonstrations:
- Teleoperation data collection
- Learning objective functions
- One-shot learning for new objects

### Foundation Models

Vision-language models for manipulation:
- Natural language instructions
- Zero-shot generalization
- RT-1, RT-2, PaLM-E
```python
# Example: using RT-2 for manipulation
instruction = "pick up the red cup"
image = camera.capture()

action = rt2_model(instruction, image)
robot.execute(action)
```

## Dual-Arm Manipulation

Coordinated bimanual tasks:

### Challenges
- Synchronization between arms
- Relative pose constraints
- Object deformation handling

### Applications
- Folding laundry
- Opening containers
- Large object manipulation
- Human-robot handover

## Benchmarks and Competitions

Standardized evaluation:
- **RoboCup @Home**: service robot tasks
- **Amazon Robotics Challenge**: warehouse picking
- **World Robot Summit**: industrial assembly
- **IROS Robotic Grasping Competition**

## Hardware Considerations

### Actuators
- DC motors: position control
- Series elastic: force sensing
- Pneumatic: compliant, high power-to-weight

### Sensors
- F/T sensors: 6-axis force-torque
- Tactile: pressure distribution
- Proximity: pre-contact detection

### End-Effector Design
- Modular quick-change systems
- Task-specific vs. general-purpose
- Weight and payload constraints

## Real-World Challenges

- Object variability and deformation
- Occlusions and clutter
- Real-time constraints
- Safety in human environments
- Cost and maintenance

## Industry Applications

### Manufacturing
- Pick and place
- Machine tending
- Quality inspection
- Packaging

### Logistics
- Order fulfillment
- Parcel sorting
- Depalletizing

### Healthcare
- Surgical assistance
- Rehabilitation therapy
- Medication dispensing

## Summary

Manipulation is central to robot utility. Progress in sensing, control, and learning is enabling more dexterous and robust manipulation. The final chapter explores how robots safely interact with humans.