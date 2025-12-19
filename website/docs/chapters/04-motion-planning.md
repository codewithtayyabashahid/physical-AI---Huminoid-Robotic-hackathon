---
id: 04-motion-planning
title: Motion Planning and Control
sidebar_position: 4
---

# Motion Planning and Control

Motion planning generates feasible trajectories for robots to reach goal configurations while avoiding obstacles and satisfying dynamic constraints.

## Configuration Space

Planning occurs in configuration space (C-space):
- Each point represents a complete robot state
- Obstacles mapped to C-space obstacles
- Path planning becomes point navigation

## Sampling-Based Planning

### RRT (Rapidly-exploring Random Trees)

Efficient exploration of high-dimensional spaces:
```python
def RRT(start, goal, obstacles, max_iterations):
    tree = Tree(start)
    for i in range(max_iterations):
        random_config = sample_random()
        nearest_node = tree.nearest(random_config)
        new_node = steer(nearest_node, random_config)
        if collision_free(nearest_node, new_node):
            tree.add(new_node)
            if near(new_node, goal):
                return construct_path(tree, goal)
    return None
```

### RRT* (Optimal RRT)
- Asymptotically optimal paths
- Rewiring step improves solution quality
- Used in motion planning libraries (OMPL)

### PRM (Probabilistic Roadmap Method)
- Builds graph of valid configurations
- Query phase connects start and goal
- Effective for multi-query scenarios

## Grid-Based Planning

### A* Algorithm
- Optimal for grid worlds
- Heuristic guides search toward goal
- Complete and optimally efficient

### D* Lite
- Incremental replanning
- Efficient for changing environments
- Used in autonomous navigation

## Trajectory Optimization

Converting paths to executable trajectories:

### Minimum Time Trajectory
- Subject to velocity and acceleration limits
- Optimal time parameterization
- Bang-bang control solutions

### Smoothness Optimization
- Minimize jerk (third derivative)
- Comfortable motion for humans
- Energy efficiency

### Direct Methods
- Discretize trajectory into waypoints
- Nonlinear optimization (IPOPT, SNOPT)
- Constraints: collisions, dynamics, limits

## Reactive Planning

Real-time obstacle avoidance:

### Dynamic Window Approach (DWA)
- Search in velocity space
- Simulates forward trajectories
- Selects best velocity command

### Artificial Potential Fields
- Attractive force toward goal
- Repulsive forces from obstacles
- Local minima problem

## Advanced Topics

### Task and Motion Planning (TAMP)
- Combines symbolic and geometric planning
- Manipulation among movable objects
- Logic-based action selection

### Model Predictive Control (MPC)
- Receding horizon optimization
- Handles constraints explicitly
- Real-time trajectory generation

## Implementation Example
```python
import ompl.base as ob
import ompl.geometric as og

def plan_path(start, goal, robot, obstacles):
    space = ob.SE3StateSpace()
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(-10)
    bounds.setHigh(10)
    space.setBounds(bounds)
    
    si = ob.SpaceInformation(space)
    si.setStateValidityChecker(
        ob.StateValidityCheckerFn(is_valid)
    )
    
    planner = og.RRTstar(si)
    planner.setProblemDefinition(pdef)
    planner.solve(1.0)
    
    return pdef.getSolutionPath()
```

## Summary

Motion planning bridges perception and actuation. Modern planners handle high-dimensional spaces, dynamic obstacles, and real-time constraints. Next, we explore how machine learning enhances robotic capabilities.