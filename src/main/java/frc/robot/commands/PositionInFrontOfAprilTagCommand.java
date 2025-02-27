package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class PositionInFrontOfAprilTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final AprilTagVisionSubsystem visionSubsystem;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    public PositionInFrontOfAprilTagCommand(DriveSubsystem driveSubsystem, AprilTagVisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem);

        // Initialize PID controllers
        xController = new PIDController(0.1, 0.0, 0.0);
        yController = new PIDController(0.1, 0.0, 0.0);
        rotationController = new PIDController(1.0, 0.0, 0.0);

        // Set tolerances for the PID controllers
        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        rotationController.setTolerance(0.1);
    }

    @Override
    public void initialize() {
        // Initialization code if needed
    }

    @Override
    public void execute() {
        try {
            Pose3d tagPose = visionSubsystem.aprilTagPose();
            Pose3d robotPose = visionSubsystem.calculateRobotPose();

            // Calculate errors
            Translation2d translationError = tagPose.getTranslation().toTranslation2d().minus(robotPose.getTranslation().toTranslation2d());
            //1, 1
            double rotationError = tagPose.getRotation().getZ() - robotPose.getRotation().getZ();
            //180

            // Calculate the PID outputs
            double xSpeed = xController.calculate(translationError.getX(), 0);
            double ySpeed = yController.calculate(translationError.getY(), 0);
            double rotationSpeed = rotationController.calculate(rotationError, 0);

            // Adjust the drive subsystem to correct the position
            driveSubsystem.drive(xSpeed, ySpeed, 0, true);
        } catch (Exception e) {
            // Handle the exception if no AprilTag is detected
            SmartDashboard.putString("apriltag error", e.getMessage());
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
        return xController.atSetpoint() && yController.atSetpoint() && rotationController.atSetpoint();
    }
}