package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private CameraVision cameraVision;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        try {
            cameraVision = new CameraVision(0, 1); // Assuming camera indices 0 and 1
            SmartDashboard.putString("CameraVision Status", "Initialized successfully");
        } catch (Exception e) {
            SmartDashboard.putString("CameraVision Init Error", e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        try {
            if (cameraVision != null) {
                cameraVision.detectTagsAndBalls();
            }
        } catch (Exception e) {
            SmartDashboard.putString("CameraVision Error", e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void disabledInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}
}