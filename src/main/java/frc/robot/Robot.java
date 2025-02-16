package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AprilTagVisionSubsystem;
import frc.robot.Library.*;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmSubsystem;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private AprilTagVisionSubsystem m_visionSubsystem;
   
    
    private boolean hasSeen = true;
    private double currentPoint;
    private double targetPoint;
    private double difference;
    // final DigitalInput sensor = new DigitalInput(0);

    private boolean hasPressed = false;

    private FakePS4Controller operatorController = new FakePS4Controller(1);
    private FakePS4Controller driverController = new FakePS4Controller(0);

    ArmSubsystem Arm = new ArmSubsystem();
    ElevatorSubsystem Elevator = new ElevatorSubsystem();

    TalonFX cc = new TalonFX(21);

    @Override
    public void robotInit() {
       m_robotContainer = new RobotContainer();
        String leftCameraName = "leftCam";
        String rightCameraName = "rightCam";
        Transform3d cameraToRobot = new Transform3d(new Translation3d(0.15, 0, 0), new Rotation3d(0, 0, 0));
        m_visionSubsystem = new AprilTagVisionSubsystem(leftCameraName, rightCameraName, cameraToRobot);
        
        
        
        currentPoint = 0;
        targetPoint = 0;
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
        Arm.Armmotor.set_P(1);
        Arm.Armmotor.set_I(0);
        Arm.Armmotor.set_D(0);
        Arm.Armmotor.SetZero();
        Arm.Armmotor.set_allowedClosedLoopError(0.05);
        Arm.Armmotor.set_maxVelocity(500);
        Arm.Armmotor.set_maxAcceleration(2500);
  
        Elevator.ElevatorMotor.set_P(1);
        Elevator.ElevatorMotor.set_I(0);
        Elevator.ElevatorMotor.set_D(0);
        Elevator.ElevatorMotor.SetZero();;
        Elevator.ElevatorMotor.set_allowedClosedLoopError(0.15);
        Elevator.ElevatorMotor.set_maxVelocity(2500);
        Elevator.ElevatorMotor.set_maxAcceleration(2500*5);
    }

    @Override
    public void teleopPeriodic() {
        //----------------------------------------------------------------
        // Elevator
        //----------------------------------------------------------------
        if (driverController.getTriangleButton()){
            //scoring top pose
            Arm.Setposition(12); // 1 = 50 DEGREES
            Elevator.Setposition(34);
          }else if(driverController.getSquareButton()){
            //score mid pose
            Arm.Setposition(12);
            Elevator.Setposition(0);
          }
          else if(driverController.getCircleButton()){
            //pickup pose
            Arm.Setposition(-2.8);
            Elevator.Setposition(37.5);
          }else if(driverController.getCrossButton()){
            //down pose
            Arm.Setposition(0);
            Elevator.Setposition(0);
          }
      
          if(driverController.getR1Button()){
            cc.set(0.3);
          }else if(driverController.getL1Button()){
            cc.set(-0.3);
          }else{
            cc.set(0);
          }
          
          if(driverController.getOptionsButton()){
            Arm.Stop();
            Elevator.Stop();
          }
        
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