---
id: 09-sensors-actuators
title: Sensors and Actuators
sidebar_position: 9
---

# Sensors and Actuators

The physical components that enable robots to sense and act in the world are fundamental to embodied intelligence. This chapter explores the hardware that bridges computation and physical reality.

## Sensors: Robot Perception Hardware

### Vision Sensors

#### RGB Cameras
Standard color imaging:
- CMOS vs CCD sensors
- Resolution vs frame rate trade-offs
- Rolling vs global shutter
- Lens selection: focal length, distortion
- Auto-exposure and white balance

#### Depth Cameras
3D sensing technologies:

**Stereo Cameras**
- Two cameras mimic binocular vision
- Baseline determines depth range
- Block matching for disparity
- Examples: ZED, RealSense D435

**Structured Light**
- Project known pattern
- Measure deformation
- High accuracy at short range
- Intel RealSense SR300

**Time-of-Flight (ToF)**
- Measure light travel time
- Fast acquisition (30+ fps)
- Limited by ambient light
- Microsoft Kinect Azure
```python
import pyrealsense2 as rs

# Configure RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

# Get depth and color frames
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()
```

#### Event Cameras
Neuromorphic vision:
- Asynchronous pixel events
- Microsecond temporal resolution
- High dynamic range (>120dB)
- Low latency and power
- DVS, DAVIS sensors

### Range Sensors

#### LiDAR (Light Detection and Ranging)

**2D LiDAR**
- Single plane scanning
- Used for mobile robot navigation
- Hokuyo, SICK, RPLidar
- Range: 10-30m typical

**3D LiDAR**
- Multiple scanning planes
- Point cloud generation
- Velodyne, Ouster, Livox
- Automotive and drone applications

**Solid-State LiDAR**
- No moving parts
- Compact and reliable
- Flash LiDAR, OPA, MEMS
- Future of autonomous vehicles

#### Ultrasonic Sensors
- Low cost distance measurement
- 2cm to 4m typical range
- Parking sensors, obstacle detection
- Wide beam pattern

#### Infrared (IR) Sensors
- Active IR: emitter + detector
- Sharp GP2 series common
- Affected by surface properties
- Short range (10-80cm)

### Inertial Measurement Units (IMUs)

Components:
- **Accelerometers**: linear acceleration (3-axis)
- **Gyroscopes**: angular velocity (3-axis)
- **Magnetometers**: heading reference

**MEMS IMUs**
- Small, low-cost, integrated
- MPU6050, BNO055, ICM-20948
- Drift accumulation over time
- Sensor fusion required

**High-Performance IMUs**
- Fiber optic gyros (FOG)
- Ring laser gyros (RLG)
- Used in aerospace, military
- Expensive but accurate

**Sensor Fusion**
```python
import numpy as np

class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.angle = 0.0
    
    def update(self, gyro, accel, dt):
        # Integrate gyroscope (high-pass)
        gyro_angle = self.angle + gyro * dt
        
        # Use accelerometer for long-term (low-pass)
        accel_angle = np.arctan2(accel[1], accel[2])
        
        # Complementary filter
        self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
        return self.angle
```

### Force and Torque Sensors

**6-Axis F/T Sensors**
- Measure 3 forces + 3 torques
- Strain gauge based
- ATI, OnRobot, Robotiq
- Essential for manipulation

**Load Cells**
- Single-axis force measurement
- Compression, tension, or both
- Gripper force feedback
- Overload protection

### Tactile Sensors

**Resistive Sensors**
- Force-sensitive resistors (FSR)
- Pressure changes resistance
- Simple and low-cost
- Non-linear response

**Capacitive Sensors**
- Detect proximity and touch
- Multi-touch capability
- Used in touchscreens
- Drift over temperature

**Optical Tactile Sensors**
- Camera inside soft material
- GelSight, DIGIT
- High spatial resolution
- Texture and geometry sensing

### Environmental Sensors

- **Temperature**: thermocouples, RTDs
- **Humidity**: capacitive, resistive
- **Gas**: CO2, VOC, smoke detection
- **Sound**: MEMS microphones
- **Light**: photodiodes, photoresistors

### Proprioceptive Sensors

**Encoders**
- Measure joint angles
- Optical: incremental, absolute
- Magnetic: Hall effect, AMR
- Resolution vs cost trade-off

**Potentiometers**
- Analog position sensing
- Simple but wear-prone
- Used in hobby servos

## Actuators: Robot Motion Hardware

### Electric Motors

#### DC Motors

**Brushed DC Motors**
- Simple control (voltage = speed)
- Commutator and brushes
- Maintenance required
- Low cost

**Brushless DC Motors (BLDC)**
- Electronic commutation
- Higher efficiency
- Longer lifetime
- Requires ESC (Electronic Speed Controller)
```python
# Simple BLDC control via PWM
import RPi.GPIO as GPIO

PWM_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(PWM_PIN, GPIO.OUT)

pwm = GPIO.PWM(PWM_PIN, 50)  # 50Hz
pwm.start(0)

# Set speed (0-100%)
def set_motor_speed(speed):
    duty_cycle = speed
    pwm.ChangeDutyCycle(duty_cycle)
```

#### Stepper Motors

**Types**
- Permanent magnet
- Variable reluctance
- Hybrid

**Characteristics**
- Precise positioning without feedback
- Holding torque at standstill
- Step resolution: full, half, microstepping
- Prone to missed steps under load

**Control**
- Step and direction signals
- Motor drivers: A4988, DRV8825
- Used in 3D printers, CNC

#### Servo Motors

**Hobby Servos**
- Integrated DC motor + gearbox + controller
- PWM control (1-2ms pulse)
- Limited range (typically 180°)
- Common in humanoid robotics

**Industrial Servos**
- High-performance closed-loop control
- Encoder feedback
- Brushless design
- Expensive but precise

### Pneumatic Actuators

**Advantages**
- High power-to-weight ratio
- Inherent compliance
- Explosion-proof
- Fast response

**Disadvantages**
- Requires air compressor
- Difficult precise control
- Noisy operation
- Air consumption

**Types**
- Linear cylinders
- Rotary actuators
- Soft pneumatic actuators
- McKibben artificial muscles

### Hydraulic Actuators

**Characteristics**
- Highest force capability
- Used in heavy machinery
- Boston Dynamics Atlas
- Complex maintenance

**Components**
- Hydraulic pump
- Control valves
- Cylinders or motors
- Hydraulic fluid reservoir

### Shape Memory Alloys (SMA)

**Properties**
- Deform when heated
- Return to original shape
- Nitinol (NiTi) common
- Used in medical devices

**Limitations**
- Slow actuation (heating/cooling)
- Low efficiency
- Fatigue after cycles
- Difficult to control precisely

### Series Elastic Actuators (SEA)

**Design**
- Spring between motor and load
- Measure spring deflection for force
- Safer for human interaction
- Used in prosthetics, exoskeletons

**Advantages**
- Inherent compliance
- Force control without sensor
- Energy storage and return
- Shock tolerance

### Soft Actuators

**Pneumatic Soft Actuators**
- Flexible materials (silicone, rubber)
- Inflate to generate motion
- Bending, extending, twisting
- Safe human interaction

**Dielectric Elastomer Actuators**
- Voltage causes deformation
- Artificial muscles
- High strain capability
- Still experimental

### Gearboxes and Transmissions

**Gear Reduction**
- Increase torque, reduce speed
- Spur, planetary, harmonic gears
- Backdrivability vs efficiency

**Backdrivable Designs**
- Important for collaboration
- Low friction and inertia
- Quasi-direct drive

**Harmonic Drives**
- High reduction ratio (50:1 to 200:1)
- Compact and precise
- Zero backlash
- Expensive

## Sensor-Actuator Integration

### Closed-Loop Control

Feedback control architecture:
```
Sensor → Controller → Actuator → Physical System
   ↑                                      ↓
   └──────────────────────────────────────┘
```

### Real-Time Constraints

- Control loop frequencies: 100Hz - 1kHz
- Sensor sampling rates
- Actuator bandwidth
- Communication latency

### Safety Circuits

- Emergency stop (E-stop) buttons
- Limit switches
- Watchdog timers
- Redundant sensors

## Power Systems

### Battery Technologies

**Lithium Polymer (LiPo)**
- High energy density
- Lightweight
- Requires careful charging
- Common in drones, mobile robots

**Lithium-Ion**
- Cylindrical cells (18650, 21700)
- Safer than LiPo
- Tesla, laptops, power tools
- Good cycle life

**Power Management**
- Battery Management System (BMS)
- Voltage regulation
- Current limiting
- Charging circuits

### Power Distribution

- Multiple voltage rails (3.3V, 5V, 12V, 24V)
- Isolation for noise-sensitive circuits
- Fusing and circuit protection
- Power budgeting

## Communication Interfaces

### Digital Protocols

**I2C (Inter-Integrated Circuit)**
- Two-wire serial (SDA, SCL)
- Multiple devices on bus
- 100kHz to 400kHz typical
- Short distance (< 1m)

**SPI (Serial Peripheral Interface)**
- Full-duplex communication
- Higher speed than I2C
- More wires required
- Common for sensors, displays

**UART (Universal Asynchronous Receiver-Transmitter)**
- Simple serial communication
- TX, RX, GND
- Configurable baud rate
- RS-232, RS-485 variants

**CAN Bus**
- Robust for automotive
- Multi-master protocol
- Error detection
- Used in mobile robots

### Wireless Communication

- **WiFi**: high bandwidth, medium range
- **Bluetooth**: low power, short range
- **Zigbee**: mesh networking, IoT
- **LoRa**: long range, low data rate
- **5G**: future of robot connectivity

## Hardware Design Considerations

### Mechanical Design

- Weight distribution and balance
- Structural strength vs weight
- Thermal management
- Vibration dampening
- Modularity and serviceability

### Material Selection

- **Metals**: aluminum, steel, titanium
- **Polymers**: ABS, nylon, PEEK
- **Composites**: carbon fiber, fiberglass
- **Soft materials**: silicone, rubber

### Manufacturing Methods

- **Subtractive**: CNC machining, laser cutting
- **Additive**: 3D printing (FDM, SLA, SLS)
- **Casting**: injection molding
- **Assembly**: fasteners, adhesives, welding

## Calibration and Maintenance

### Sensor Calibration

- Zero-offset correction
- Scale factor adjustment
- Cross-axis sensitivity
- Temperature compensation

### Actuator Tuning

- PID parameter tuning
- Backlash compensation
- Friction identification
- Thermal characterization

### Preventive Maintenance

- Regular cleaning
- Lubrication schedules
- Wear inspection
- Firmware updates

## Emerging Technologies

- **Soft robotics**: compliant actuators and sensors
- **Biocompatible materials**: implantable sensors
- **Energy harvesting**: self-powered sensors
- **Neuromorphic hardware**: event-based sensing
- **Quantum sensors**: ultra-precise measurements

## Summary

Sensors and actuators are the physical interface between algorithms and the real world. Understanding their capabilities and limitations is essential for designing capable robotic systems. The next chapter explores simulation and digital twins for robot development.