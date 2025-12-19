---
id: 11-integration
title: System Integration and Architecture
sidebar_position: 11
---

# System Integration and Architecture

Building complete robotic systems requires integrating perception, planning, control, and hardware into a cohesive architecture. This chapter covers software frameworks, communication, and deployment.

## Robot Software Architecture

### Layered Architecture

Classic three-layer model:

**Deliberative Layer (Planning)**
- High-level decision making
- Path planning
- Task scheduling
- Model-based reasoning

**Executive Layer (Control)**
- Motion control
- Behavior arbitration
- Resource management
- Real-time constraints

**Reactive Layer (Reflexes)**
- Obstacle avoidance
- Safety monitoring
- Emergency stops
- Low-latency responses

### Subsumption Architecture

Behavior-based robotics:
- Multiple parallel behaviors
- Priority-based arbitration
- No central planning
- Reactive and robust

### Hybrid Architectures

Combining deliberative and reactive:
- 3T architecture
- LAAS architecture
- Modern approach in most robots

## ROS: Robot Operating System

Industry-standard middleware:

### ROS 2 Fundamentals

**Nodes**

Modular processes:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

**Topics**
- Publish-subscribe messaging
- Many-to-many communication
- Asynchronous
- Different QoS profiles

**Services**
- Request-response pattern
- Synchronous communication
- Client-server model
```python
from example_interfaces.srv import AddTwoInts

def add_two_ints_callback(request, response):
    response.sum = request.a + request.b
    return response

service = node.create_service(
    AddTwoInts, 
    'add_two_ints', 
    add_two_ints_callback
)
```

**Actions**
- Long-running tasks
- Feedback during execution
- Preemptable
- Navigation, manipulation

### ROS 2 Tools

**Command Line**
```bash
# List nodes
ros2 node list

# View topics
ros2 topic list
ros2 topic echo /odom
ros2 topic hz /scan

# Service call
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# Parameter management
ros2 param get /node_name param_name
ros2 param set /node_name param_name value
```

**Visualization**
- RViz: 3D visualization
- RQt: GUI tools
- PlotJuggler: data plotting

**Recording and Playback**
```bash
# Record data
ros2 bag record /topic1 /topic2

# Playback
ros2 bag play bagfile.db3

# Info
ros2 bag info bagfile.db3
```

### ROS Packages

Standard packages:
- **navigation2**: autonomous navigation
- **MoveIt**: manipulation planning
- **ros2_control**: hardware abstraction
- **tf2**: coordinate transformations
- **perception_pcl**: point cloud processing

## Real-Time Considerations

### Real-Time Operating Systems

**Requirements**
- Deterministic response times
- Priority scheduling
- Low latency
- Preemptive multitasking

**Options**
- **PREEMPT_RT Linux**: real-time patched kernel
- **Xenomai**: dual-kernel approach
- **QNX**: commercial RTOS
- **VxWorks**: aerospace/defense

### Real-Time in ROS 2

**DDS (Data Distribution Service)**
- Real-time middleware
- QoS policies
- Multiple vendors (Fast-DDS, Cyclone DDS)

**Real-Time Executor**
```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  
  // Create real-time executor
  auto executor = std::make_shared
    rclcpp::executors::StaticSingleThreadedExecutor>();
  
  auto node = std::make_shared<MyNode>();
  executor->add_node(node);
  
  // Run with real-time priority
  executor->spin();
  
  rclcpp::shutdown();
  return 0;
}
```

### Control Loop Timing

**Jitter Management**
- Fixed timestep loops
- Deadline monitoring
- Priority inversion prevention
- CPU affinity
```python
import time

def control_loop(frequency=100):
    """Fixed-rate control loop"""
    period = 1.0 / frequency
    next_time = time.time()
    
    while True:
        # Control computation
        control_output = compute_control()
        send_to_actuators(control_output)
        
        # Wait for next cycle
        next_time += period
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            print(f"Warning: missed deadline by {-sleep_time:.3f}s")
```

## State Management

### State Machines

Behavior specification:
```python
from enum import Enum

class RobotState(Enum):
    IDLE = 0
    MOVING = 1
    GRASPING = 2
    ERROR = 3

class StateMachine:
    def __init__(self):
        self.state = RobotState.IDLE
    
    def transition(self, event):
        if self.state == RobotState.IDLE:
            if event == "start_move":
                self.state = RobotState.MOVING
        
        elif self.state == RobotState.MOVING:
            if event == "reached_target":
                self.state = RobotState.GRASPING
            elif event == "error":
                self.state = RobotState.ERROR
```

### SMACH (ROS State Machine)
```python
import smach

# Define states
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['outcome1','outcome2'],
                           input_keys=['foo_input'],
                           output_keys=['foo_output'])

    def execute(self, userdata):
        # State logic
        if userdata.foo_input > 5:
            return 'outcome1'
        else:
            return 'outcome2'

# Create state machine
sm = smach.StateMachine(outcomes=['succeeded','failed'])

with sm:
    smach.StateMachine.add('FOO', Foo(),
                          transitions={'outcome1':'BAR',
                                     'outcome2':'succeeded'})
    smach.StateMachine.add('BAR', Bar(),
                          transitions={'outcome1':'FOO',
                                     'outcome2':'failed'})

# Execute
outcome = sm.execute()
```

## Communication Protocols

### Ethernet and UDP/TCP

**Advantages**
- High bandwidth
- Existing infrastructure
- Flexible topology

**ROS DDS**
- Built on UDP multicast
- Automatic discovery
- QoS configuration

### CAN Bus

Automotive standard:
- Multi-master
- Message prioritization
- Error detection
- Used in mobile robots
```python
import can

# Initialize CAN interface
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Send message
msg = can.Message(arbitration_id=0x123,
                 data=[0, 25, 0, 1, 3, 1, 4, 1],
                 is_extended_id=False)
bus.send(msg)

# Receive message
message = bus.recv(timeout=1.0)
print(f"Received: {message}")
```

### EtherCAT

Real-time Ethernet:
- Deterministic
- High performance
- Used in industrial robots
- Complex setup

## Data Logging and Monitoring

### Logging Best Practices

**Log Levels**
```python
import logging

logger = logging.getLogger('robot')

logger.debug("Detailed information for debugging")
logger.info("Normal operation")
logger.warning("Unexpected but handled event")
logger.error("Error that affects functionality")
logger.critical("System failure imminent")
```

**Structured Logging**
```python
import structlog

logger = structlog.get_logger()

logger.info("robot_moved",
           position=[1.0, 2.0, 3.0],
           velocity=0.5,
           timestamp=time.time())
```

### Metrics and Telemetry

**Prometheus**
- Time-series metrics
- Alerting
- Grafana dashboards
```python
from prometheus_client import Counter, Gauge, Histogram

# Define metrics
grasps_total = Counter('grasps_total', 'Total grasp attempts')
battery_voltage = Gauge('battery_voltage', 'Battery voltage')
control_latency = Histogram('control_latency_seconds', 'Control loop latency')

# Update metrics
grasps_total.inc()
battery_voltage.set(12.3)
with control_latency.time():
    compute_control()
```

### Remote Monitoring

**Web Dashboards**
- Foxglove Studio
- WebViz
- Custom React/Vue dashboards

**Alert Systems**
- Email/SMS notifications
- PagerDuty integration
- Slack webhooks

## Testing and Validation

### Unit Testing
```python
import unittest

class TestKinematics(unittest.TestCase):
    def setUp(self):
        self.robot = RobotArm()
    
    def test_forward_kinematics(self):
        joint_angles = [0, 0, 0, 0, 0, 0]
        pose = self.robot.forward_kinematics(joint_angles)
        expected_pose = [0.5, 0, 0.3, 0, 0, 0]
        self.assertAlmostEqual(pose, expected_pose, places=3)
    
    def test_inverse_kinematics(self):
        target_pose = [0.4, 0.1, 0.2, 0, 0, 0]
        joint_angles = self.robot.inverse_kinematics(target_pose)
        # Verify solution
        computed_pose = self.robot.forward_kinematics(joint_angles)
        self.assertAlmostEqual(computed_pose, target_pose, places=2)
```

### Integration Testing
```python
def test_pick_and_place():
    """End-to-end test of pick and place"""
    # Setup
    robot.move_to_home()
    object_pose = detect_object()
    
    # Execute task
    robot.move_to(object_pose)
    robot.grasp()
    robot.move_to(target_location)
    robot.release()
    
    # Verify
    assert object_at(target_location)
    assert gripper_is_open()
```

### Simulation-Based Testing
```python
import pytest

@pytest.mark.simulation
def test_navigation_obstacle_avoidance():
    """Test navigation in simulated environment"""
    sim = Simulator()
    sim.load_world("warehouse.world")
    robot = sim.spawn_robot([0, 0, 0])
    
    # Add obstacle
    sim.spawn_object("box", [2, 0, 0])
    
    # Command navigation
    result = robot.navigate_to([5, 0, 0])
    
    # Verify
    assert result.success
    assert robot.get_pose()[0] > 4.5
    assert not robot.collision_occurred()
```

### Continuous Integration

**GitHub Actions**
```yaml
name: CI

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Build ROS package
        run: |
          source /opt/ros/humble/setup.bash
          colcon build
      - name: Run tests
        run: |
          colcon test
          colcon test-result --verbose
```

## Deployment

### Containerization

**Docker for Robotics**
```dockerfile
FROM ros:humble

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-moveit \
    python3-pip

# Copy robot code
COPY ./robot_ws /root/robot_ws

# Build workspace
WORKDIR /root/robot_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Entry point
CMD ["bash", "-c", "source install/setup.bash && ros2 launch robot_bringup robot.launch.py"]
```

### Fleet Management

**Multi-Robot Coordination**
- Central orchestrator
- Task allocation
- Collision avoidance
- Load balancing

**Over-the-Air Updates**
- Staged rollouts
- Rollback capability
- Version tracking
- Delta updates

### Edge Computing

**On-Robot Processing**
- NVIDIA Jetson: embedded GPU
- Intel NUC: compact x86
- Raspberry Pi: low-cost ARM

**Cloud Offloading**
- Heavy computation in cloud
- Low-latency local control
- Hybrid architectures

## Security

### Authentication and Authorization

- API tokens
- Certificate-based auth
- Role-based access control (RBAC)
- Secure boot

### Network Security

- VPN for remote access
- Firewall rules
- Encrypted communication (TLS)
- Intrusion detection

### Safety-Critical Systems

- Formal verification
- Redundancy
- Watchdog timers
- Safe failure modes

## Case Study: Full System Architecture

### Autonomous Mobile Manipulator

**Hardware Layer**
- Mobile base (differential drive)
- 6-DOF manipulator arm
- RGB-D camera
- 2D LiDAR
- IMU

**Software Stack**
```
┌─────────────────────────────────────┐
│     Task Planning & Execution       │  (Python)
├─────────────────────────────────────┤
│  Navigation  │  Manipulation        │  (ROS 2 Nav2, MoveIt)
├──────────────┴──────────────────────┤
│    Perception & State Estimation    │  (C++/Python)
├─────────────────────────────────────┤
│       Hardware Abstraction          │  (ros2_control)
├─────────────────────────────────────┤
│          Hardware Drivers           │  (C++)
└─────────────────────────────────────┘
```

**Communication Flow**
1. User command to Task planner
2. Task planner to Motion planners
3. Motion planners to Controllers
4. Controllers to Hardware drivers
5. Sensors to Perception to State estimator
6. State estimator provides Feedback to planners

## Summary

System integration is where components become a cohesive robot. Proper architecture, communication, testing, and deployment practices are essential for reliable robotic systems. The final chapter explores deployment and real-world considerations.