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

import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private AprilTagVisionSubsystem m_visionSubsystem;
   
    //sensor to look at the coral
    // final DigitalInput sensor = new DigitalInput(0);


    private FakePS4Controller operatorController = new FakePS4Controller(1);
    private FakePS4Controller driverController = new FakePS4Controller(0);

    //Arm subsystem
    ArmSubsystem Arm = new ArmSubsystem();
    String ArmPosition = "Zero";

    //Elevator subsystem
    ElevatorSubsystem Elevator = new ElevatorSubsystem();
    String LiftPosition = "Zero";

    TalonFX cc = new TalonFX(21);

    @Override
    public void robotInit() {
       m_robotContainer = new RobotContainer();
        String leftCameraName = "leftCam";
        String rightCameraName = "rightCam";
        Transform3d cameraToRobot = new Transform3d(new Translation3d(0.15, 0, 0), new Rotation3d(0, 0, 0));
        m_visionSubsystem = new AprilTagVisionSubsystem(leftCameraName, rightCameraName, cameraToRobot);
        
        //Initialising the arm
        Arm.Armmotor.set_P(1);//1
        Arm.Armmotor.set_I(0);//0
        Arm.Armmotor.set_D(0);//0
        Arm.Armmotor.SetZero();
        Arm.Armmotor.set_allowedClosedLoopError(0.075);//0.075
        Arm.Armmotor.set_maxVelocity(100);//500
        Arm.Armmotor.set_maxAcceleration(500);//2500
        //Initialising the Elevator
        Elevator.ElevatorMotor.set_P(1);//1
        Elevator.ElevatorMotor.set_I(0);//0
        Elevator.ElevatorMotor.set_D(0);//0
        Elevator.ElevatorMotor.SetZero();;
        Elevator.ElevatorMotor.set_allowedClosedLoopError(0.15);//0.15
        Elevator.ElevatorMotor.set_maxVelocity(3000);//2500
        Elevator.ElevatorMotor.set_maxAcceleration(3000*5);// 2500*5
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
    }

    @Override
    public void teleopPeriodic() {
        //----------------------------------------------------------------
        // Elevator & Arm
        // Cross = down position (home position)
        // Square = mid position (L2)
        // Triangle = high position (L3)
        // ... = top position (L4)
        // Circle = intaking position (coral station)
        // ... = low position (L1)
        //----------------------------------------------------------------
        if (driverController.getTriangleButton()){
            //scoring high pose
            Arm.Setposition(90); // 12
            Elevator.Setposition(231); //34

            LiftPosition = "L3";
            ArmPosition = "Scoring";
        }else if(driverController.getSquareButton()){
            //score mid pose
            Arm.Setposition(140); //12
            Elevator.Setposition(0);

            LiftPosition = "L2";
            ArmPosition = "Scoring";
        }else if(driverController.getCircleButton()){
            //pickup pose
            Arm.Setposition(-36); // -2.8
            Elevator.Setposition(300); //37.5

            LiftPosition = "Coral Station";
            ArmPosition = "Intaking";
        }else if(driverController.getCrossButton()){
            //down pose
            Arm.Setposition(0);
            Elevator.Setposition(0);

            LiftPosition = "Home";
            ArmPosition = "Home";
        }
      
          
        if(driverController.getOptionsButton()){
            Arm.Stop();
            Elevator.Stop();
        }

        //----------------------------------------------------------------
        // Coral Cannon
        // Holding R1 = Intaking coral
        // Holding L1 = Outtaking coral
        //----------------------------------------------------------------
        if(driverController.getR1Button()){
            // Spitting coral out
            cc.set(0.3);
        }else if(driverController.getL1Button()){
            // Taking coral in
            cc.set(-0.3);
        }else{
            // Set motor 
            cc.set(0);
        }


        SmartDashboard.putString("LiftPose", LiftPosition);
        SmartDashboard.putNumber("LiftPoint", Elevator.ElevatorMotor.STS.get_position());
        
        SmartDashboard.putString("ArmPose", ArmPosition);
        SmartDashboard.putNumber("ArmPoint", Arm.Armmotor.STS.get_position());

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