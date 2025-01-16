package frc.robot;



import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
        PhotonCamera cameraLeft = new PhotonCamera("leftCam");
        PhotonCamera cameraRight = new PhotonCamera("rightCam");

        

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        var leftResult = cameraLeft.getLatestResult();
        var rightResult = cameraRight.getLatestResult();
        boolean hasResult = leftResult.hasTargets() && rightResult.hasTargets();
    String leftString = leftResult.toString();
    String rightString = rightResult.toString();
        SmartDashboard.putString("links", leftString);
        SmartDashboard.putString("rechts", rightString);
        SmartDashboard.putBoolean("hasResult", hasResult);
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