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
import frc.robot.Library.*;

import frc.robot.subsystems.*;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private AprilTagVisionSubsystem m_visionSubsystem;

    private ElevatorSubsystem Elevator = RobotContainer.Lift;
    private ArmSubsystem Arm = RobotContainer.Arm;
    private ScoreSubsystem CoralCannon = RobotContainer.cc;

   
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

    //pressed
    boolean haspressed = false;
    
   
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        String leftCameraName = "leftCam";
        String rightCameraName = "rightCam";
        Transform3d cameraToRobot = new Transform3d(new Translation3d(0.15, 0, 0), new Rotation3d(0, 0, 0));
        m_visionSubsystem = new AprilTagVisionSubsystem(leftCameraName, rightCameraName, cameraToRobot);

        // Arm
        Arm.Motor.SetZero();

        // Elevator
        Elevator.Motor.SetZero();
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
            Elevator.Stop();
            Elevator.Zero();
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

        // Switch with Coral scoring & Algea scoring
        if(haspressed == false && driverController.getR1ButtonPressed()){
            haspressed = true;
        }else if(haspressed == true && driverController.getR1ButtonPressed()){
            haspressed = false;
        }
        
        //----------------------------------------------------------------
        // Coral Positions
        //----------------------------------------------------------------
        SmartDashboard.putBoolean("pressed", haspressed);
        if (driverController.getCrossButton() && haspressed == false){
            //L1 pose
            Arm.Setposition(-40); // 12
            Elevator.Setposition(0); //34
            
            LiftPosition = "L1";
            ArmPosition = "L1Scoring";
        }else if(driverController.getSquareButton() && haspressed == false){
            //L2 pose
            Arm.Setposition(145); //12
            Elevator.Setposition(0);

            LiftPosition = "L2";
            ArmPosition = "L2Scoring";
        }else if(driverController.getTriangleButton() && haspressed == false){
            //L3 pose
            Arm.Setposition(145); // -2.8
            Elevator.Setposition(220); //37.5

            LiftPosition = "L3";
            ArmPosition = "L3Scoring";
        }else if(driverController.getCircleButton() && haspressed == false){
            //L4 pose 
            Arm.Setposition(165);
            Elevator.Setposition(525);

            LiftPosition = "L4";
            ArmPosition = "L4Scoring";
        }
        

        else if (driverController.getCrossButton() && haspressed == true){
            //Algae Low
            Arm.Setposition(75);
            Elevator.Setposition(75);
            
            LiftPosition = "Algae low";
            ArmPosition = "Algae";
        }else if(driverController.getSquareButton() && haspressed == true){
            //Algae high
            Arm.Setposition(75);
            Elevator.Setposition(250);

            LiftPosition = "Algae high";
            ArmPosition = "Algae";
        }else if(driverController.getTriangleButton() && haspressed == true){
            //Algae net
            Arm.Setposition(145); // -2.8
            Elevator.Setposition(220); //37.5

            LiftPosition = "Net";
            ArmPosition = "Net";
        }else if(driverController.getCircleButton() && haspressed == true){
            //Algae processor
            Arm.Setposition(165);
            Elevator.Setposition(525);

            LiftPosition = "Processor";
            ArmPosition = "Processor";
        }

        else if(driverController.getPOV() > 0){ //180 is down
            //Home pose
        
            Arm.Setposition(0);
            Elevator.Setposition(0);

            LiftPosition = "Home";
            ArmPosition = "Home";
        }else if(driverController.getL1Button()){
            //Intake Pose

            Arm.Setposition(-28);
            Elevator.Setposition(260);

            LiftPosition = "Coral Station";
            ArmPosition = "Intake";
        }  
          
        if(driverController.getOptionsButton()){
            Arm.Stop();
            Elevator.Stop();
        }
    
        //----------------------------------------------------------------
        // Coral Cannon
        // Holding R2 = Intaking coral
        // Holding L2 = Outtaking coral
        //----------------------------------------------------------------
        if(driverController.getR2Axis() > 0){
            // Spitting coral out
            CoralCannon.setPower(0.3);
            // TODO Automatisation after shooting out
        }else if(driverController.getL2Axis() > 0){
            // Taking coral in
            CoralCannon.setPower(-0.3);
            // Automatic Stop Corral
            if(CoralCannon.CoralInPosition()){
                CoralCannon.setPower(0);
            }

        }else{
            // Set motor 
            CoralCannon.setPower(0);
        }
    

        SmartDashboard.putString("LiftPose", LiftPosition);
        SmartDashboard.putNumber("LiftPoint", Elevator.Motor.STS.get_position());
        
        SmartDashboard.putString("ArmPose", ArmPosition);
        SmartDashboard.putNumber("ArmPoint", Arm.Motor.STS.get_position());

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