---
id: 05-machine-learning
title: Machine Learning for Robotics
sidebar_position: 5
---

# Machine Learning for Robotics

Machine learning enables robots to learn from experience, adapt to new situations, and improve performance over time. This chapter covers key ML techniques for robotics.

## Reinforcement Learning

RL trains agents through trial and error by maximizing cumulative rewards.

### Markov Decision Processes (MDPs)

Foundation of RL:
- States (s): robot configurations
- Actions (a): control commands
- Rewards (r): task performance feedback
- Policy π(a|s): mapping states to actions
- Value function V(s): expected future reward

### Deep Q-Networks (DQN)

Value-based RL using neural networks:
```python
import torch
import torch.nn as nn

class DQN(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, action_dim)
    
    def forward(self, state):
        x = torch.relu(self.fc1(state))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)
```

### Policy Gradient Methods

Directly optimize the policy:
- REINFORCE: Monte Carlo policy gradient
- Actor-Critic: combines value and policy learning
- A3C: asynchronous parallel training
- Advantage function reduces variance

### Proximal Policy Optimization (PPO)

State-of-the-art policy gradient method:
- Clipped objective prevents large policy updates
- Stable and sample efficient
- Used by OpenAI for robotic manipulation
- Works well with continuous control

### Soft Actor-Critic (SAC)

Maximum entropy RL:
- Encourages exploration
- Off-policy algorithm (sample efficient)
- Automatic temperature tuning
- Excellent for robotic control tasks

## Imitation Learning

Learning from expert demonstrations:

### Behavioral Cloning

Supervised learning from expert data:
- Collect expert trajectories
- Train policy: π(a|s) = f(s; θ)
- Fast but suffers from distribution shift
- Requires large datasets

### Dataset Aggregation (DAgger)

Addresses distribution shift:
1. Train policy on expert data
2. Collect new data using learned policy
3. Query expert for corrections
4. Aggregate datasets and retrain
5. Iterate until convergence

### Inverse Reinforcement Learning (IRL)

Learn reward function from demonstrations:
- Infer expert's objectives
- Enables transfer to new situations
- Maximum entropy IRL
- Generative Adversarial Imitation Learning (GAIL)

## Learning from Simulation

Simulation accelerates learning and reduces hardware costs:

### Sim-to-Real Transfer

Challenges and solutions:
- **Reality gap**: differences between sim and real
- **Domain randomization**: vary physics parameters
- **Domain adaptation**: fine-tune on real data
- **Realistic rendering**: bridge visual gap

### Popular Simulators

- **MuJoCo**: physics engine for robotics
- **PyBullet**: open-source alternative
- **Isaac Gym**: GPU-accelerated parallel simulation
- **Gazebo**: full robot simulation with ROS
```python
import mujoco
import mujoco_viewer

model = mujoco.MjModel.from_xml_path('robot.xml')
data = mujoco.MjData(model)

viewer = mujoco_viewer.MujocoViewer(model, data)
while viewer.is_alive:
    mujoco.mj_step(model, data)
    viewer.render()
```

## Vision-Based Learning

Learning directly from pixels:

### Convolutional Neural Networks

Extract visual features:
- Object detection for grasping
- Semantic segmentation for navigation
- Depth estimation from monocular images

### End-to-End Learning

Direct policy from images:
- Input: camera images
- Output: control commands
- Challenges: sample complexity, interpretability
- Success: autonomous driving, drone racing

### Vision-Language Models for Robotics

Foundation models for robotics:
- **RT-1**: Robotics Transformer from Google
- **PaLM-E**: multimodal language model
- **RT-2**: vision-language-action model
- Natural language task specification

## Self-Supervised Learning

Learning without labels:

### Contrastive Learning

Learn representations from unlabeled data:
- SimCLR, MoCo for visual representations
- Time-contrastive learning for robotics
- Useful for transfer learning

### World Models

Learn predictive models of environment:
- Dynamics model: predict next state
- Enables planning in latent space
- Sample-efficient RL

## Meta-Learning

Learning to learn:
- Few-shot adaptation to new tasks
- Model-Agnostic Meta-Learning (MAML)
- Fast adaptation with limited data
- Multi-task robotics scenarios

## Practical Considerations

### Data Collection

- Teleoperation for initial data
- Automated data generation
- Quality over quantity
- Diversity in scenarios

### Safety During Learning

- Initial training in simulation
- Gradual real-world deployment
- Safety constraints (CBF, shield policies)
- Human oversight and intervention

### Evaluation Metrics

- Success rate on test tasks
- Sample efficiency (episodes to converge)
- Generalization to novel scenarios
- Robustness to perturbations

## Case Studies

### Robotic Grasping

- DeepMind's QT-Opt: vision-based grasping
- Google's Everyday Robots: large-scale learning
- 1000+ hours of trial and error

### Bipedal Locomotion

- Learning to walk in simulation
- Transfer to real humanoids (Cassie, Digit)
- Robust to pushes and uneven terrain

### Autonomous Driving

- Waymo: ML for perception and planning
- Tesla: end-to-end neural networks
- Millions of miles of driving data

## Summary

Machine learning has revolutionized robotics, enabling capabilities previously impossible with hand-coded controllers. RL, imitation learning, and foundation models are pushing boundaries. The next chapter explores humanoid locomotion, a challenging application of these techniques.