# Team Epsilon 11100 - 2026 Robot Code

<div align="center">

![FRC 2026](https://img.shields.io/badge/FRC-2026-blue)
![Team](https://img.shields.io/badge/Team-11100-red)
![WPILib](https://img.shields.io/badge/WPILib-2026.2.1-green)
![Java](https://img.shields.io/badge/Java-17-orange)

**Advanced Swerve Drive Robot with Vision-Based Auto-Aiming**

</div>

---

## 📋 Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Hardware Configuration](#hardware-configuration)
- [Subsystems](#subsystems)
- [Autonomous Commands](#autonomous-commands)
- [Control Scheme](#control-scheme)
- [Software Architecture](#software-architecture)
- [Setup & Deployment](#setup--deployment)
- [Configuration](#configuration)
- [Development](#development)

---

## 🎯 Overview

This repository contains the competition robot code for **Team Epsilon 11100** for the 2026 FRC season. The robot features a sophisticated swerve drive system with triple-camera vision processing, automated shooter alignment, and advanced ballistic trajectory calculations for precise scoring.

### Technology Stack

- **Framework**: WPILib 2026.2.1
- **Language**: Java 17
- **Motor Controllers**: CTRE Phoenix 6 (TalonFX)
- **Vision**: PhotonVision with 3 cameras
- **Logging**: AdvantageKit
- **Build System**: Gradle

---

## ✨ Key Features

### 🚗 Advanced Swerve Drive
- **Field-centric drive** with alliance-aware orientation
- **Heading lock** - maintains robot orientation during translation
- **Input curves** (cubic) for precise control
- **Slow mode** for fine positioning (35% speed reduction)
- **Vision-fused odometry** for accurate pose estimation

### 🎯 Autonomous Aiming System
- **Real-time ballistic trajectory solving** with physics-based calculations
- **Predictive turret aiming** with motion compensation
- **Continuous auto-elevation** (hood angle) adjustment
- **Dual-axis automated targeting** (elevation + yaw)
- **Vision-based AprilTag tracking** with 3-camera coverage

### 📷 Vision System
- **Triple-camera setup** (Main, Left, Right) for 270° field coverage
- **Multi-tag pose fusion** via Kalman filtering
- **Dynamic trust scoring** based on distance, ambiguity, and tag count
- **Automatic pose correction** integrated with drivetrain odometry

### ⚙️ Precision Shooter System
- **Flywheel velocity control** (up to 6000 RPM)
- **Hood angle adjustment** (20-56°) with software limits
- **Turret rotation** (±180°) with cable wrap protection
- **Ballistic solver** optimized for <1ms computation time
- **Safe fallback modes** when vision unavailable

---

## 🔧 Hardware Configuration

### Motor IDs (Organized by System)

| System | Component | Motor ID | Notes |
|--------|-----------|----------|-------|
| **Shooter (3x)** | Hood | 31 | Position control (20-56°) |
| | Flywheel | 32 | Velocity control (0-6000 RPM) |
| | Turret | 33 | Position control (±180°) |
| **Intake (4x)** | Intake Wheels | 41 | Velocity control (5500 RPM) |
| | Pivot | 42 | Position control (deploy/retract) |
| **Indexer (5x)** | Indexer | 51 | Velocity control (5500 RPM) |
| **Kicker (6x)** | Kicker | 61 | Velocity control (4000 RPM) |

### Vision Cameras

| Camera | Position | Offset (X, Y, Z) | Yaw | Trust Factor |
|--------|----------|------------------|-----|--------------|
| **Main** | Front-center | (12.28", 12.31", 16.16") | 0° | 1.0 (highest) |
| **Left** | Front-left | (12", 12", 9.3") | -45° | 1.5 |
| **Right** | Front-right | (12", -12", 9.3") | +45° | 1.5 |

### Software Limits

All mechanisms have dual safety layers:
1. **Hardware software limits** (TalonFX controller enforced)
2. **Software clamping** (code-level bounds checking)

---

## 🤖 Subsystems

### 1. Drivetrain (`Drivetrain.java`)
**Type**: Swerve Drive (Phoenix 6 Tuner X Generated)

**Features**:
- Field-centric drive with heading lock
- Vision pose fusion via `SwerveDrivePoseEstimator`
- Real-time velocity tracking (vx, vy, omega)
- Alliance-aware field orientation
- AdvantageKit logging integration

**Key Methods**:
```java
getPose()               // Returns fused odometry + vision pose
getFieldVelocities()    // Returns [vx, vy, omega] for predictive aiming
addVisionMeasurement()  // Integrates camera observations
```

---

### 2. Vision (`Vision.java`)
**Type**: Multi-camera AprilTag tracking

**Features**:
- Processes 3 PhotonVision cameras simultaneously
- Filters detections by ambiguity (<0.7) and Z-error (<1.0m)
- Calculates trust scores based on distance, skew, and tag count
- Returns closest visible tag for aiming

**Key Methods**:
```java
getClosestVisibleTag(Pose2d robotPose)  // Returns nearest AprilTag
getLatestVisionPose()                    // Returns last computed robot pose
```

**Filtering Logic**:
- Ambiguity < 0.7 (single-tag poses)
- Z-error < 1.0m (height consistency)
- Distance-weighted trust scoring
- Multi-tag detection bonus (0.5x std dev)

---

### 3. Shooter System

#### Hood (`Hood.java`)
**Control**: Motion Magic Expo (position)

**Range**: 20-56° (0.5-12.5 motor rotations)

**Features**:
- Continuous angle adjustment during auto-aim
- Hardware + software limit enforcement
- Angle-to-position conversion with gear ratio

---

#### Flywheel (`Flywheel.java`)
**Control**: Motion Magic Velocity

**Range**: 0-6000 RPM

**Features**:
- Feedforward control (kS=0.555, kV=0.127, kA=0.1)
- Always commanded to valid RPM (≥1000) with safe fallback
- Real-time RPM, current, and voltage logging

---

#### Turret (`Turret.java`)
**Control**: Motion Magic Expo (position)

**Range**: ±180° (±50 motor rotations with 100:1 gearing)

**Features**:
- Predictive aiming with robot velocity compensation
- Cable wrap prevention via software limits
- Robot-relative angle control (0° = forward)

---

### 4. Intake (`Intake.java`)
**Components**: Intake wheels + pivot

**Features**:
- **Deploy/Retract**: Pivot motor position control
- **Pump Mode**: Oscillates between deployed (1.0) and pumped (0.5) positions
- **Velocity Control**: 5500 RPM intake speed (Slot 1 PID)

**Positions**:
- Deployed: 1.0 rotations (fully extended)
- Pumped: 0.5 rotations (partial retract)
- Retracted: 0.0 rotations (stored)

---

### 5. Indexer (`Indexer.java`)
**Control**: Velocity (5500 RPM)

**Purpose**: Stage game pieces for shooting

---

### 6. Kicker (`Kicker.java`)
**Control**: Velocity (4000 RPM)

**Purpose**: Final acceleration before shooter exit

---

## 🎮 Control Scheme

### Xbox Controller (Port 0)

| Input | Function | Notes |
|-------|----------|-------|
| **Left Stick** | Drive (X/Y) | Field-centric translation |
| **Right Stick** | Rotation | Manual rotation or heading lock |
| **Left Bumper** | Slow Mode | 35% speed reduction |
| **Right Trigger** | Kicker Activate | Runs at 4000 RPM while held |
| **Left Trigger** | Pump Intake | Oscillates deploy/pump positions |

### Automated Systems (Always Active)
- **Hood**: Auto-aims elevation based on closest AprilTag
- **Turret**: Auto-aims yaw with predictive motion compensation
- **Flywheel**: Maintains ballistically-calculated RPM

---

## 🤖 Autonomous Commands

### AutoElevationCommand
**Purpose**: Continuous hood angle adjustment

**Operation**:
1. Get robot pose from fused odometry
2. Find closest visible AprilTag
3. Solve ballistic trajectory (angle + RPM)
4. Command hood and flywheel every 20ms cycle

**Fallback**: Maintains last valid settings when tags lost

**Performance**: <1ms solve time (0.5° angle steps)

---

### AutoYawCommand
**Purpose**: Continuous turret tracking with predictive aiming

**Operation**:
1. Get robot pose and field velocities
2. Calculate target angle using `CalculateYaw.aimWithLookahead()`
3. Compensate for robot motion during shot flight time
4. Command turret every 20ms cycle

**Lookahead Time**: 200ms (configurable in Constants)

**Fallback**: Holds last valid angle when tags lost

---

### PumpIntakeCommand
**Purpose**: Repetitive intake cycling for game piece control

**Cycle**:
1. Deploy (1.0 rotations)
2. Wait 250ms
3. Pump (0.5 rotations)
4. Wait 250ms
5. Repeat until button released

**End Behavior**: Returns to deployed position

---

### DriveCommand
**Purpose**: Advanced field-centric driving with heading lock

**Features**:
- Cubic input curves for precise control
- Automatic heading hold when no rotation input
- Alliance-aware field orientation
- Slow mode toggle (left bumper)

**Heading Lock Logic**:
- Activates when rotation input < deadband
- Holds last commanded heading
- Timeout: 100ms after rotation stops

---

## 🏗️ Software Architecture

### Command-Based Pattern
All subsystems follow WPILib's command-based architecture:
- **Subsystems**: Extend `SubsystemBase`
- **Commands**: Extend `Command` with lifecycle methods
- **Default Commands**: Run continuously (auto-aim, drive)
- **Button Bindings**: Trigger-based command execution

### Utility Classes

#### BallisticSolver (`BallisticSolver.java`)
**Purpose**: Physics-based trajectory calculation

**Method**: `solvePreferConstantRpm(x, y, z, preferredRPM, config)`

**Inputs**:
- Target position (x, y, z in meters)
- Preferred flywheel RPM (5500)
- Shooter configuration (height, diameter, exit velocity factor)

**Outputs**:
- Launch angle (degrees)
- Motor RPM
- Exit velocity (m/s)
- Flight time (seconds)

**Optimization**:
- Inlined calculations (no function calls in loop)
- Pre-calculated constants
- 0.5° angle steps (72 iterations)
- <1ms typical solve time

---

#### CalculateYaw (`CalculateYaw.java`)
**Purpose**: Turret angle calculation with motion compensation

**Method**: `aimWithLookahead(robotPos, robotYaw, vx, vy, omega, targetPos, lookaheadTime)`

**Returns**:
```java
record AimAngles(
    Rotation2d fieldAngle,          // Absolute field direction
    Rotation2d robotRelativeAngle   // Turret offset from robot forward
)
```

**Features**:
- Predicts future robot position during flight time
- Accounts for robot rotation (omega)
- Proper angle wrapping (±π)

---

### Constants Organization (`Constants.java`)

All configuration values organized by subsystem in nested static classes:

```java
public class Constants {
    public class constDrivetrain { /* max speed, dimensions, etc. */ }
    public class constVision { /* camera offsets, filtering thresholds */ }
    public class constHood { /* angle limits, motor IDs, PID gains */ }
    public class constTurret { /* gear ratio, angle limits */ }
    public class constIntake { /* positions, RPM, pump timing */ }
    public class constFlywheel { /* RPM limits, feedforward */ }
    public class constBallisticSolver { /* physics constants */ }
}
```

---

## 🚀 Setup & Deployment

### Prerequisites
- WPILib 2026.2.1 or later
- Java 17 JDK
- VS Code with WPILib extension
- PhotonVision on coprocessor (e.g., Raspberry Pi)

### Initial Setup

1. **Clone Repository**
   ```bash
   git clone https://github.com/Team-Epsilon-11100/2026.git
   cd 2026/luigi-bot
   ```

2. **Configure Team Number**
   - Edit `.wpilib/wpilib_preferences.json`
   - Set `teamNumber` to `11100`

3. **Build Project**
   ```bash
   ./gradlew build
   ```

4. **Deploy to Robot**
   ```bash
   ./gradlew deploy
   ```

### PhotonVision Setup

1. **Configure Camera Names** (must match Constants.java):
   - MainCamera
   - LeftCamera
   - RightCamera

2. **Calibrate Cameras**:
   - Use PhotonVision calibration tool
   - Export and save calibration files

3. **Network Configuration**:
   - Coprocessor static IP: `10.111.0.11` (recommended)
   - Robot radio: `10.111.0.1`

---

## ⚙️ Configuration

### Critical Constants to Adjust

#### Vision (`constVision`)
```java
// Camera physical offsets (measure from robot center)
mainCameraOffset = new Transform3d(...)
leftCameraOffset = new Transform3d(...)
rightCameraOffset = new Transform3d(...)

// Filtering thresholds
maxAmbiguity = 0.7  // Increase if rejecting too many tags
maxZError = 1.0     // Height error tolerance (meters)
```

#### Ballistic Solver (`constBallisticSolver`)
```java
shooterHeightMeters = 17.069"  // Measure from floor to shooter exit
flywheelDiameterMeters = 4"    // Actual wheel diameter
exitVelocityFactor = 0.85      // Tune: ball speed / wheel surface speed
preferredFlywheelRPM = 5500    // Target shooting speed
angleStepDeg = 1.0             // Solver resolution (0.5-2.0° recommended)
```

#### PID Gains
Tune for each mechanism in respective `const*` classes:
- `constHood.kP = 2.0` (position control)
- `constFlywheel.kP = 0.2` (velocity control with feedforward)
- `constTurret.kP = 2.0` (position control)

### Test Mode vs Competition Mode

**Testing**: Aims directly at AprilTag center
```java
constAutoAim.useTagCenterForTesting = true;
```

**Competition**: Aims at fixed goal height
```java
constAutoAim.useTagCenterForTesting = false;
constAutoAim.absoluteGoalHeightMeters = Units.inchesToMeters(72);
```

---

## 🛠️ Development

### Project Structure
```
luigi-bot/
├── src/main/java/frc/robot/
│   ├── commands/          # Command implementations
│   │   ├── AutoElevationCommand.java
│   │   ├── AutoYawCommand.java
│   │   ├── DriveCommand.java
│   │   └── PumpIntakeCommand.java
│   ├── subsystems/        # Hardware subsystems
│   │   ├── drivetrain/
│   │   ├── vision/
│   │   ├── hood/
│   │   ├── flywheel/
│   │   ├── turret/
│   │   ├── intake/
│   │   ├── indexer/
│   │   └── kicker/
│   ├── utils/             # Helper utilities
│   │   ├── BallisticSolver.java
│   │   └── CalculateYaw.java
│   ├── Constants.java     # Configuration constants
│   ├── RobotContainer.java # Subsystem wiring & bindings
│   └── Main.java / Robot.java
├── vendordeps/            # Third-party dependencies
│   ├── Phoenix6-26.1.1.json
│   ├── photonlib.json
│   └── AdvantageKit.json
└── build.gradle           # Build configuration
```

### Adding a New Subsystem

1. **Create Subsystem Class**:
   ```java
   public class MySubsystem extends SubsystemBase {
       public MySubsystem() {
           // Initialize hardware
       }
       
       @Override
       public void periodic() {
           // Log to SmartDashboard
       }
   }
   ```

2. **Add Constants**:
   ```java
   public class constMySubsystem {
       public static final int motorId = 70;
       public static final double kP = 1.0;
   }
   ```

3. **Instantiate in RobotContainer**:
   ```java
   private final MySubsystem mySubsystem;
   
   public RobotContainer() {
       mySubsystem = new MySubsystem();
   }
   ```

### Debugging Tips

#### SmartDashboard Keys
Monitor these values during operation:

**Vision**:
- `Vision/TotalVisibleTags` - Should be > 0 when tags in view
- `Vision/ConnectedCameras` - Should be 3
- `Vision/MainCamera_RejectionReason` - Debug filtering issues

**Auto-Aiming**:
- `AutoElev/Status` - "Tracking" / "No tags visible"
- `AutoElev/TargetAngle` - Hood angle command
- `AutoElev/TargetRPM` - Flywheel RPM command
- `AutoYaw/TargetAngle` - Turret angle command

**Motors**:
- `Flywheel/RPM` - Actual vs target RPM
- `Hood/AngleDeg` - Current hood angle
- `Turret/AngleDeg` - Current turret angle

#### Common Issues

**"No tags available"**:
- Check `Vision/ConnectedCameras` = 3
- Verify camera names in PhotonVision match Constants
- Increase `maxAmbiguity` if filtering too strict

**Flywheel not spinning**:
- Check motor ID (32)
- Verify motor connection and LED status
- Monitor `Flywheel/Current` for activity

**Hood jerking randomly**:
- Reduce PID gains (`constHood.kP`)
- Check angle limits (20-56°)
- Verify ballistic solver returning valid solutions

---

## 📊 Performance Metrics

- **Ballistic Solve Time**: <1ms (optimized)
- **Vision Update Rate**: 30 Hz (per camera)
- **Odometry Update Rate**: 250 Hz (CAN FD)
- **Command Scheduler**: 50 Hz (20ms period)
- **Pose Fusion Latency**: <50ms (vision to drivetrain)

---

## 📝 License

This project is licensed under the WPILib BSD License. Portions derived from:
- **WPILib** - Copyright (c) FIRST and other WPILib contributors
- **AdvantageKit** - Copyright 2021-2025 FRC 6328 (Mechanical Advantage)

---

## 🤝 Contributing

### Team Members
- **Programming Lead**: [Name]
- **Vision Lead**: [Name]
- **Drive Team**: [Names]

### Development Workflow
1. Create feature branch from `main`
2. Test thoroughly in simulation
3. Deploy to practice robot
4. Submit pull request with description
5. Code review by programming lead
6. Merge after approval

---

## 📞 Contact

**Team Epsilon 11100**
- GitHub: [Team-Epsilon-11100](https://github.com/Team-Epsilon-11100)
- Email: [team@example.com]
- Website: [teamepsilon.org]

---

<div align="center">

**Built with ❤️ by Team Epsilon 11100**

*Good luck at competition! 🏆*

</div>
