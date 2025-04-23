# Buildingseason2025 - Ghost Foundation Robotics
![Static Badge](https://img.shields.io/badge/FIRST%20Robotics%20Competition%20-%20%23105D95?style=for-the-badge&logo=first&logoColor=white&logoSize=auto&labelColor=%23105D95)
![GitHub last commit](https://img.shields.io/github/last-commit/GhostFoundation/Buildingseason2025%2FComp-new-end-effector?style=for-the-badge)

Welcome to the official codebase for Ghost Foundation's FRC 2025 competition robot. This repository contains all the logic, configuration, and subsystem control for our swerve-driven robot, tailored for the 2025 FIRST Robotics Competition.

# table of contents
1. [Overview](#overview)
2. [Project Structure](#project-structure)
3. [Subsystems & Key Components](#subsystems)
5. [Change Log format](#change-log-format)
6. [Change Log](#change-log)

## Overview

This project is built using Java and the WPILib framework. It features:
- **Swerve Drive** using MAXSwerve modules.
- **Arm and Elevator** mechanisms for manipulating game pieces.
- **Subsystem-based** architecture using WPILib’s Command-based design pattern.
- **Advanced PID control** using custom speed controllers.
- **Vision processing** through the Limelight camera.

[![Watch the video](https://img.youtube.com/vi/tZvk1nJOGS0/0.jpg)]([https://www.youtube.com/watch?v=_5tFXJQIzi4](https://www.youtube.com/watch?v=tZvk1nJOGS0))

## Project Structure
```
Buildingseason2025/
│
├── build.gradle                                      # Gradle build configuration
│
└── src/
    └── main/
        └── java/
            └── frc/
                └── robot/
                    ├── Commands/                     # Command classes for autonomous and teleop behavior
                    │   ├── Drive/                         # Drive-related commands
                    │   ├── Arm/                           # Arm movement logic
                    │   ├── Elevator/                      # Elevator motion control
                    │   └── Score/                         # Scoring actions and sequences
                    │
                    ├── Library/                      # Custom motor and PID control classes
                    │   ├── NEO_SpeedCtrl.java            # Wrapper for NEO motor control with encoders
                    │   ├── TalonFX_PID.java              # PID controller for Falcon (TalonFX) motors
                    │   └── NEO_Absolute_PID.java         # Absolute encoder PID control for NEOs
                    │
                    ├── subsystems/                   # Robot subsystems, representing physical components
                    │   ├── DriveSubsystem.java
                    │   ├── ArmSubsystem.java
                    │   ├── ElevatorSubsystem.java
                    │   └── ScoreSubsystem.java
                    │
                    ├── Constants.java                # All global constants (ports, PID values, tuning parameters)
                    ├── Configs.java                  # Loadable robot configuration profiles
                    ├── LimelightHelpers.java         # Utility class for vision processing with Limelight
                    ├── Robot.java                    # Main robot entry point (extends TimedRobot)
                    └── RobotContainer.java           # Central binding class for subsystems and commands
```

## Subsystems

- **DriveSubsystem**
  - Implements swerve drive using `MAXSwerveModule`.
  - Integrates gyro feedback and odometry for precise movement.
  
- **ArmSubsystem**
  - Controls a rotating arm with PID control using `TalonFX_PID`.
  
- **ElevatorSubsystem**
  - Manages vertical lift for game pieces, with adjustable speed and position control.

- **ScoreSubsystem**
  - Coordinates actions for scoring game pieces.

- **Vision**
  - Integrated via `LimelightHelpers` for autonomous targeting and pathing.

## Custom Utilities

- **NEO_SpeedCtrl**
  - Custom speed controller for NEO motors, utilizing encoders for feedback and precise control.

- **TalonFX_PID & NEO_Absolute_PID**
  - PID wrappers for Falcon (TalonFX) motors and NEO motors, allowing for fine-tuned motion control.

## Autonomous & Teleop

The robot’s autonomous and teleop modes are managed through the `RobotContainer.java`, where commands are registered and scheduled using WPILib's Command-based design pattern. Configuration for autonomous modes is handled via constants and runtime environment settings.

## Building & Deploying

To build and deploy the project, you need to have the following:

- **WPILib** and the **FRC VSCode Extension** installed.
- **Gradle** installed, or you can use WPILib’s built-in Gradle wrapper.

Once these dependencies are set up, you can build the project and deploy it to the robot using the standard FRC deployment process.

## Change Log Format

The Change Log is formatted as follows:

**DD-MM-YYYY**

- **Responsible:** [Name of the Person]
- **Task:** A brief description of the task performed.
- **Details:** Additional context if necessary.
- **Info:** Optional details for others to consider when continuing the work.

## Change Log

**14-01-2025**

- **Responsible:** Gijs van Maanen
- **Task:** Create basic drive code with gyro.
- **Details:** Swerve drive works, field-oriented driving (gyro) is functional.
- **Info:** Max velocity set at 3 m/s for training; real max speed is 7.22 m/s.

**18-01-2025**

- **Responsible:** Gijs van Maanen
- **Task:** Implement AprilTag detection with the Orange Pi (PhotonVision).
- **Details:** AprilTag detection implemented in the branch [apriltag-vision-by-Gijs](https://github.com/GhostFoundation/Buildingseason2025/tree/apriltag-vision-by-Gijs). Chose not to implement algae detection due to limited info and camera requirements. Discussion pending.
- **Info:** Needs testing.

**21-01-2025**

- **Responsible:** Gijs van Maanen
- **Task:** Merged AprilTag branch with main.
- **Details:** AprilTag vision tested successfully and merged with the main branch.
