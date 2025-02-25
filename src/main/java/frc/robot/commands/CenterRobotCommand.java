package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class CenterRobotCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final AprilTagVisionSubsystem visionSubsystem;

    public CenterRobotCommand(DriveSubsystem driveSubsystem, AprilTagVisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void initialize() {
        // Initialization code if needed
    }

    @Override
    public void execute() {
        try {
            var robotPose = visionSubsystem.calculateRobotPose();
            // Logic to center the robot based on the robotPose
            // For example, you can use the x and y coordinates to adjust the robot's position
            double xError = robotPose.getX();
            double yError = robotPose.getY();
            double rotationError = robotPose.getRotation().getZ();

            // Adjust the drive subsystem to correct the position
            driveSubsystem.drive(-xError, -yError, -rotationError, false);
        } catch (Exception e) {
            // Handle the exception if no AprilTag is detected
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        driveSubsystem.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        // Define the condition to finish the command
        return false;
    }
}