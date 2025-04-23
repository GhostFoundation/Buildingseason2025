# Buildingseason2025
Ghost foundation Buildingseason repo of 2025

# table of contents
1. [Info about flowcharts](#info-about-flowcharts)
2. [Change Log format](#change-log-format)
3. [Change Log](#change-log)


# info about flowcharts
Please use these symbols with their meaning:
![flowchartRules](https://github.com/user-attachments/assets/d9ad3125-5951-4d97-b8a5-b935cfcd5127)

If you finished your flowchart please add it to this folder: [Flowcharts](https://github.com/GhostFoundation/Buildingseason2025/tree/main/flowcharts)

# Buildingseason2025 - Ghost Foundation Robotics

Welcome to the official codebase for Ghost Foundation's FRC 2025 competition robot. This repository contains all the logic, configuration, and subsystem control for our swerve-driven robot, tailored for the 2025 FIRST Robotics Competition.

## 🚀 Overview

This project is built using Java and the WPILib framework. It features:
- **Swerve Drive** using MAXSwerve modules.
- **Arm and Elevator** mechanisms for manipulating game pieces.
- **Subsystem-based** architecture using WPILib’s Command-based design pattern.
- **Advanced PID control** using custom speed controllers.
- **Vision processing** through the Limelight camera.

## 🧩 Project Structure

Buildingseason2025/ ├── src/ │ └── main/java/frc/robot/ │ ├── Commands/ # Command classes for each subsystem │ ├── Library/ # Custom controller and PID utilities │ ├── subsystems/ # Subsystems: Drive, Arm, Elevator, etc. │ ├── Constants.java # Centralized constants used project-wide │ ├── Configs.java # Configuration loader for robot tuning │ ├── LimelightHelpers.java # Helper functions for Limelight integration │ ├── Robot.java # Main robot class (entry point) │ └── RobotContainer.java# Binds subsystems and commands ├── build.gradle # Build configuration using Gradle └── README.md # You're here!

## ⚙️ Subsystems & Key Components

- **DriveSubsystem**
  - Implements swerve drive using `MAXSwerveModule`.
  - Integrates gyro feedback and odometry.
- **ArmSubsystem**
  - Controls a rotating arm with PID control using `TalonFX_PID`.
- **ElevatorSubsystem**
  - Manages vertical lift for game pieces.
- **ScoreSubsystem**
  - Coordinates actions to score game pieces effectively.
- **Vision**
  - Integrated via `LimelightHelpers`, assists in targeting and autonomous pathing.

## 🔧 Custom Utilities

- **NEO_SpeedCtrl**
  - Speed controller for NEO motors using encoders.
- **TalonFX_PID & NEO_Absolute_PID**
  - PID wrappers around respective motor types with tuning configurations.

## 🧠 Autonomous & Teleop

Commands are registered and scheduled via `RobotContainer.java` using a Command-based structure. Auton mode configurations are loaded based on constants and runtime environment.

## 🛠️ Building & Deploying

Make sure you have:
- WPILib and FRC VSCode Extension installed.
- Gradle installed or use WPILib's built-in Gradle wrapper.


### Change Log format:
DD-MM-YYYY

Responsible: [Name of the Person]

- Task: Brief description of the task performed.
- Details: Additional context if required.
- Info: Anything someone else might need to think about when continuing the work (optional).

# Change Log

14-01-2025 

Responsible: Gijs van Maanen 
- Task: Create basic drivecode with gyro
- Details: swervedrive works, fieldorientated driving (gyro) works
- Info: max velocity is set at 3m/s for training purposes real max speed is 7.22 m/s


18-01-2025

Responsible: Gijs van Maanen
- Taks: Implement apriltag detection with the orange pi (photonvision)
- Details: implemented apriltag detection in branch: [apriltag-vision-by-Gijs](https://github.com/GhostFoundation/Buildingseason2025/tree/apriltag-vision-by-Gijs) decided to not do algea detection since it requires a third camera and we can only get the following info: where is the algea in 2d and how many algea are visible. This info doesnt seem too important, will be discussed with others.
- Info: Still has to be tested.

21-01-2025

Responsible: Gijs van Maanen

- Task: Merged apriltag branch with main
- Details: Tested apriltag vision, it works so i merged it with main
