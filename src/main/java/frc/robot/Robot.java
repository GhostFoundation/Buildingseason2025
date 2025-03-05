package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Library.*;

import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    ScoreSubsystem CoralCannon;
    private AprilTagVisionSubsystem m_visionSubsystem;
    private final Timer CoralTimer = new Timer();
   
    //sensors
    // final DigitalInput sensor = new DigitalInput(0);
    final DigitalInput touch1 = new DigitalInput(0);
    final DigitalInput touch2 = new DigitalInput(1);

    private FakePS4Controller driverController = new FakePS4Controller(0);
    private FakePS4Controller operatorController = new FakePS4Controller(1);

    static CameraServer TunnelVision;

    //Arm subsystem
    // ArmSubsystem Arm = new ArmSubsystem();
    String ArmPosition = "Zero";

    //Elevator subsystem
    //ElevatorSubsystem Elevator = new ElevatorSubsystem();
    String LiftPosition = "Zero";
    boolean hasTouched = false;

    //pressed
    boolean haspressed = false;
    
    boolean playerss = false;

    // CoralCanon
    boolean CoralInserted = false;
    Boolean TimerTrigger = false;
   
    @Override
    public void robotInit() {
       m_robotContainer = new RobotContainer();
        String leftCameraName = "leftCam";
        String rightCameraName = "rightCam";
        Transform3d cameraToRobot = new Transform3d(new Translation3d(0.15, 0, 0), new Rotation3d(0, 0, 0));
        m_visionSubsystem = new AprilTagVisionSubsystem(leftCameraName, rightCameraName, cameraToRobot);
        CoralCannon = m_robotContainer.cc;
        CameraServer.startAutomaticCapture();
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
        //----------------------------------------------------------------
        // General
        // - DpadDown = Home
        // - Option   = Stop  
        // - L1       = Intake    
        //
        // Coral Mode               | Algae Mode
        // - Cross    = L1          | - Cross = Algae Low
        // - Square   = L2          | - Square = Algae High
        // - Triangle = L3          | - Triangle = Net (not needed)
        // - Circle   = L4          | - Circle = Processor (not needed)
        
        

/* 
        if(haspressed == false && operatorController.getR1ButtonPressed()){
            haspressed = true;
        }else if(haspressed == true && operatorController.getR1ButtonPressed()){
            haspressed = false;
        }
       
        if (operatorController.getCrossButton() && haspressed == false){
            //L1 pose
            m_robotContainer.Arm.Setposition(-40); // 12
            m_robotContainer.Lift.Setposition(-1); //34
            
            LiftPosition = "L1";
            ArmPosition = "L1Scoring";
        }else if(operatorController.getSquareButton() && haspressed == false){
            //L2 pose
            m_robotContainer.Arm.Setposition(145); //12
            m_robotContainer.Lift.Setposition(-1);

            LiftPosition = "L2";
            ArmPosition = "L2Scoring";
        }else if(operatorController.getTriangleButton() && haspressed == false){
            //L3 pose
            m_robotContainer.Arm.Setposition(145); // -2.8
            m_robotContainer.Lift.Setposition(240); //37.5

            LiftPosition = "L3";
            ArmPosition = "L3Scoring";
        }else if(operatorController.getCircleButton() && haspressed == false){
            //L4 pose 
            m_robotContainer.Arm.Setposition(165);
            m_robotContainer.Lift.Setposition(525);

            LiftPosition = "L4";
            ArmPosition = "L4Scoring";
        }
        

        else if (operatorController.getCrossButton() && haspressed == true){
            //Algae Low
            m_robotContainer.Arm.Setposition(70);
                m_robotContainer.Lift.Setposition(75);
            
            LiftPosition = "Algae low";
            ArmPosition = "Algae";
        }else if(operatorController.getSquareButton() && haspressed == true){
            //Algae high
            m_robotContainer.Arm.Setposition(70);
            m_robotContainer.Lift.Setposition(300);

            LiftPosition = "Algae high";
            ArmPosition = "Algae";
        }else if(operatorController.getTriangleButton() && haspressed == true){
            //Algae net
            m_robotContainer.Arm.Setposition(70); // -2.8
            m_robotContainer.Lift.Setposition(525); //37.5

            LiftPosition = "Net";
            ArmPosition = "Net";
        }else if(operatorController.getCircleButton() && haspressed == true){
            //Algae processor
            m_robotContainer.Arm.Setposition(20);
            m_robotContainer.Lift.Setposition(-1);

            LiftPosition = "Processor";
            ArmPosition = "Processor";
        }

        else if(operatorController.getPOV() > 0){ //180 is down
            //Home pose
        
            m_robotContainer.Arm.Setposition(0);
            m_robotContainer.Lift.Setposition(-1);

            LiftPosition = "Home";
            ArmPosition = "Home";
        }else if(operatorController.getL1Button()){
            //Intake Pose

            m_robotContainer.Arm.Setposition(-28);
            m_robotContainer.Lift.Setposition(275);

            LiftPosition = "Coral Station";
            ArmPosition = "Intake";
        }
*/

        if(haspressed == false && driverController.getR1ButtonPressed()){
            haspressed = true;
        }else if(haspressed == true && driverController.getR1ButtonPressed()){
            haspressed = false;
        }
       
        if (driverController.getCrossButton() && haspressed == false){
            //L1 pose
            m_robotContainer.Arm.Setposition(-40); // 12
            m_robotContainer.Lift.Setposition(-5); //34
            
            LiftPosition = "L1";
            ArmPosition = "L1Scoring";
        }else if(driverController.getSquareButton() && haspressed == false){
            //L2 pose
            m_robotContainer.Arm.Setposition(145); //12
            m_robotContainer.Lift.Setposition(-5);

            LiftPosition = "L2";
            ArmPosition = "L2Scoring";
        }else if(driverController.getTriangleButton() && haspressed == false){
            //L3 pose
            m_robotContainer.Arm.Setposition(160); // -2.8
            m_robotContainer.Lift.Setposition(170); //37.5

            LiftPosition = "L3";
            ArmPosition = "L3Scoring";
        }else if(driverController.getCircleButton() && haspressed == false){
            //L4 pose 
            m_robotContainer.Arm.Setposition(170);
            m_robotContainer.Lift.Setposition(480);

            LiftPosition = "L4";
            ArmPosition = "L4Scoring";
        }
        

        else if (driverController.getCrossButton() && haspressed == true){
            //Algae Low
            m_robotContainer.Arm.Setposition(70);
                m_robotContainer.Lift.Setposition(75);
            
            LiftPosition = "Algae low";
            ArmPosition = "Algae";
        }else if(driverController.getSquareButton() && haspressed == true){
            //Algae high
            m_robotContainer.Arm.Setposition(70);
            m_robotContainer.Lift.Setposition(300);

            LiftPosition = "Algae high";
            ArmPosition = "Algae";
        }else if(driverController.getTriangleButton() && haspressed == true){
            //Algae net
            m_robotContainer.Arm.Setposition(70); // -2.8
            m_robotContainer.Lift.Setposition(525); //37.5

            LiftPosition = "Net";
            ArmPosition = "Net";
        }else if(driverController.getCircleButton() && haspressed == true){
            //Algae processor
            m_robotContainer.Arm.Setposition(20);
            m_robotContainer.Lift.Setposition(-5);

            LiftPosition = "Processor";
            ArmPosition = "Processor";
        }

        else if(driverController.getPOV() > 0){ //180 is down
            //Home pose
        
            m_robotContainer.Arm.Setposition(0);
            m_robotContainer.Lift.Setposition(-5);

            LiftPosition = "Home";
            ArmPosition = "Home";
        }else if(driverController.getL1Button()){
            //Intake Pose

            m_robotContainer.Arm.Setposition(-15);//-28
            m_robotContainer.Lift.Setposition(295);

            LiftPosition = "Coral Station";
            ArmPosition = "Intake";
        }

      
          


        if(driverController.getOptionsButton()){
            m_robotContainer.Arm.Stop();
            m_robotContainer.Lift.Stop();
        }
    
        //----------------------------------------------------------------
        // Coral Cannon
        // Holding R2 = Intaking coral
        // Holding L2 = Outtaking coral
        //----------------------------------------------------------------
        // 
        if(CoralTimer.get() > 1.5 && TimerTrigger == true){
            CoralCannon.setPower(0);
            m_robotContainer.Arm.Setposition(0);
            m_robotContainer.Lift.Setposition(-5);
            
            CoralTimer.stop();
            TimerTrigger = false;
            CoralInserted = false;
        }
        
        if(driverController.getR2Axis() > 0 && CoralInserted == true && haspressed == false){
            // Automatisation after shooting out
            if (TimerTrigger == false){
                CoralTimer.reset();
                CoralTimer.start();
                TimerTrigger = true;
            }
                CoralCannon.setPower(0.3);

        //#endregion
        
        // Coral Cannon: Outtake
        //#region
        }else if(driverController.getL2Axis() > 0 && CoralInserted == false && haspressed == false){
            
            // Automatic Stop Corral
            if(CoralCannon.CoralInPosition()){
                CoralCannon.setPower(0);
                CoralInserted = true;
            }
            // Taking coral in
            else{
                CoralCannon.setPower(-0.3);
            }
        //#endregion

        // Coral Cannon: stopped
        //#region
        }else if(haspressed == false){ 
            CoralCannon.setPower(0);
        }
        //#endregion
        
        if(driverController.getR2Axis() > 0 && haspressed == true){
            CoralCannon.setPower(0.2);
        }else if(driverController.getL2Axis() > 0 && haspressed == true){
            CoralCannon.setPower(-0.5);
        }else if(haspressed == true){
            CoralCannon.setPower(0);
        }

        SmartDashboard.putBoolean("Inserted", CoralInserted);
        SmartDashboard.putNumber("Current", CoralCannon.Current());
        SmartDashboard.putNumber("Timer", CoralTimer.get());
        
        SmartDashboard.putNumber("LiftPoint", m_robotContainer.Lift.getPose());
        
        SmartDashboard.putNumber("ArmPoint", m_robotContainer.Arm.getPose());

        SmartDashboard.putBoolean("Mode", haspressed);
        SmartDashboard.putBoolean("Player", playerss);
        
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
  public void autonomousInit() {
    m_robotContainer.m_robotDrive.StraightHeading();
    m_robotContainer.Arm.Setposition(0);
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