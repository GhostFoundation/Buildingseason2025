package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
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
   
    //sensors
    // final DigitalInput sensor = new DigitalInput(0);
    final DigitalInput touch1 = new DigitalInput(0);
    final DigitalInput touch2 = new DigitalInput(1);

    //private FakePS4Controller operatorController = new FakePS4Controller(1);
    private FakePS4Controller driverController = new FakePS4Controller(0);

    //Arm subsystem
    // ArmSubsystem Arm = new ArmSubsystem();
    String ArmPosition = "Zero";

    //Elevator subsystem
    //ElevatorSubsystem Elevator = new ElevatorSubsystem();
    String LiftPosition = "Zero";
    boolean hasTouched = false;

    
   
    @Override
    public void robotInit() {
       m_robotContainer = new RobotContainer();
        String leftCameraName = "leftCam";
        String rightCameraName = "rightCam";
        Transform3d cameraToRobot = new Transform3d(new Translation3d(0.15, 0, 0), new Rotation3d(0, 0, 0));
        m_visionSubsystem = new AprilTagVisionSubsystem(leftCameraName, rightCameraName, cameraToRobot);

        //Initialising the arm
        // Arm.Armmotor.set_P(1);//1
        // Arm.Armmotor.set_I(0);//0
        // Arm.Armmotor.set_D(0);//0
        m_robotContainer.Arm.Armmotor.SetZero();
        // Arm.Armmotor.set_allowedClosedLoopError(0.075);//0.075
        // Arm.Armmotor.set_maxVelocity(100);//500
        // Arm.Armmotor.set_maxAcceleration(500);//2500
        //Initialising the Elevator
        // Elevator.ElevatorMotor.set_P(1);//1
        // Elevator.ElevatorMotor.set_I(0);//0
        // Elevator.ElevatorMotor.set_D(0);//0
        m_robotContainer.Lift.ElevatorMotor.SetZero();
        // Elevator.ElevatorMotor.set_allowedClosedLoopError(0.15);//0.15
        // Elevator.ElevatorMotor.set_maxVelocity(2500);//2500
        // Elevator.ElevatorMotor.set_maxAcceleration(2500*5);// 2500*5
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
        
        if(hasTouched == false && touch1.get()==false && touch2.get() ==false){
            m_robotContainer.Lift.Stop();
            m_robotContainer.Lift.Zero();
            hasTouched = true;
        }else if(hasTouched == true || touch1.get() == true || touch2.get() == true){
            hasTouched = false;
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
        // Cross = L1
        // Square = L2
        // Triangle = L3
        // Circle =  L4
        // DpadDown = Home
        // DpadUp = Intake    
        //----------------------------------------------------------------
        if (driverController.getCrossButton()){
            //L1 pose
            m_robotContainer.Arm.Setposition(-40); // 12
            m_robotContainer.Lift.Setposition(0); //34
            
            LiftPosition = "L1";
            ArmPosition = "L1Scoring";
        }else if(driverController.getSquareButton()){
            //L2
            m_robotContainer.Arm.Setposition(155); //12
            m_robotContainer.Lift.Setposition(0);

            LiftPosition = "L2";
            ArmPosition = "L2Scoring";
        }else if(driverController.getTriangleButton()){
            //L3
            m_robotContainer.Arm.Setposition(155); // -2.8
            m_robotContainer.Lift.Setposition(245); //37.5

            LiftPosition = "L3";
            ArmPosition = "L3Scoring";
        }else if(driverController.getCircleButton()){
            //L4 pose
            m_robotContainer.Arm.Setposition(170);
            m_robotContainer.Lift.Setposition(525);

            LiftPosition = "L4";
            ArmPosition = "L4Scoring";
        }
        
        else if(driverController.getPOV() == 180){
            //Home pose
        
            m_robotContainer.Arm.Setposition(0);
            m_robotContainer.Lift.Setposition(0);

            LiftPosition = "Home";
            ArmPosition = "Home";
        }else if(driverController.getPOV() == 0){
            //Intake Pose

            m_robotContainer.Arm.Setposition(-25);
            m_robotContainer.Lift.Setposition(225);

            LiftPosition = "Coral Station";
            ArmPosition = "Intake";
        }
      
          
        if(driverController.getOptionsButton()){
            m_robotContainer.Arm.Stop();
            m_robotContainer.Lift.Stop();
        }

        //----------------------------------------------------------------
        // Coral Cannon
        // Holding R1 = Intaking coral
        // Holding L1 = Outtaking coral
        //----------------------------------------------------------------
        if(driverController.getR1Button()){
            // Spitting coral out
            m_robotContainer.cc.setPower(0.3);
        }else if(driverController.getL1Button()){
            // Taking coral in
            m_robotContainer.cc.setPower(-0.3);
        }else{
            // Set motor 
            m_robotContainer.cc.setPower(0);
        }


        SmartDashboard.putString("LiftPose", LiftPosition);
        SmartDashboard.putNumber("LiftPoint", m_robotContainer.Lift.ElevatorMotor.STS.get_position());
        
        SmartDashboard.putString("ArmPose", ArmPosition);
        SmartDashboard.putNumber("ArmPoint", m_robotContainer.Arm.Armmotor.STS.get_position());

        //SmartDashboard.putBoolean("touch", touch1.get());
        
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