package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AprilTagVisionSubsystem;
import frc.robot.Library.*;

import frc.robot.subsystems.Elevator;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private AprilTagVisionSubsystem m_visionSubsystem;

    private Elevator elevator = new Elevator();
    private FakePS4Controller Operatorcontroller = new FakePS4Controller(1);

    @Override
    public void robotInit() {
       m_robotContainer = new RobotContainer();
        String leftCameraName = "leftCam";
        String rightCameraName = "rightCam";
        Transform3d cameraToRobot = new Transform3d(new Translation3d(0.15, 0, 0), new Rotation3d(0, 0, 0));
        m_visionSubsystem = new AprilTagVisionSubsystem(leftCameraName, rightCameraName, cameraToRobot);

    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        try {
            Pose3d robotPose = m_visionSubsystem.calculateRobotPose();
            SmartDashboard.putString("robot pose according to apriltags", robotPose.toString());
        } catch (Exception e) {
            SmartDashboard.putString("apriltag error", e.getMessage());
        }

    }

    @Override
    public void disabledInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        elevator.init();
    }

    @Override
    public void teleopPeriodic() {
        //----------------------------------------------------------------
        // Elevator
        //----------------------------------------------------------------
        // Connect controller buttons with positions
        elevator.SetZeroPosition(Operatorcontroller.getR1Button());
        elevator.SetL1(Operatorcontroller.getCrossButton());
        elevator.SetL2(Operatorcontroller.getSquareButton());
        elevator.SetL3(Operatorcontroller.getTriangleButton());
        elevator.SetL4(Operatorcontroller.getCircleButton());
        elevator.SetCorralStation(Operatorcontroller.getL1Button());
        
        // Set elevator to position depending on controller state
        elevator.SetElevator();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }
}