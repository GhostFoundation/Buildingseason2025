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
    private AprilTagVisionSubsystem m_visionSubsystem;
    private ElevatorSubsystem Elevator;
    private ArmSubsystem Arm;
    private ScoreSubsystem CoralCannon;
    private final Timer CoralTimer = new Timer();
   
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

    // CoralCanon
    boolean CoralInserted = false;
    Boolean TimerTrigger = false;
    
   
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        Elevator = m_robotContainer.Lift;
        Arm = m_robotContainer.Arm;
        CoralCannon = m_robotContainer.cc;
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
        //----------------------------------------------------------------
        // General
        // - DpadDown = Home
        // - Option   = Stop      
        //
        // Coral Mode               | Algae Mode
        // - Cross    = L1          | - Cross = Algae Low
        // - Square   = L2          | - Square = Algae High
        // - Triangle = L3          | - Triangle = Net
        // - Circle   = L4          | - Circle = Processor
        // - DpadDown = Home        |
        // - L1       = Intake      |    

        // Elevator & Arm: Coral Positions
        //#region
        if (driverController.getCrossButton() && haspressed == false){
            //L1 pose
            Arm.Set_L1(-40);
            Elevator.Set_L1(0);
        }else if(driverController.getSquareButton() && haspressed == false){
            //L2 pose
            Arm.Set_L2(145);
            Elevator.Set_L2(0);
        }else if(driverController.getTriangleButton() && haspressed == false){
            //L3 pose
            Arm.Set_L3(145);
            Elevator.Set_L3(220);
        }else if(driverController.getCircleButton() && haspressed == false){
            //L4 pose 
            Arm.Set_L4(165);
            Elevator.Set_L4(525);
        }
        else if(driverController.getL1Button()){
            //Coral station
            Arm.Set_CoralStation(-28);
            Elevator.Set_CoralStation(260);
        }  
        //#endregion
        
        // Elevator & Arm: Algae Positions
        //#region
        else if (driverController.getCrossButton() && haspressed == true){
            //Algae Low
            Arm.Set_AlgaeLow(75);
            Elevator.Set_AlgaeLow(75);
        }else if(driverController.getSquareButton() && haspressed == true){
            //Algae high
            Arm.Set_AlgaeHigh(75);
            Elevator.Set_AlgaeHigh(250);
        }else if(driverController.getTriangleButton() && haspressed == true){
            //Algae net
            Arm.Set_AlgaeNet(145);
            Elevator.Set_AlgaeNet(220);
        }else if(driverController.getCircleButton() && haspressed == true){
            //Algae processor
            Arm.Set_AlgaeProcessor(165);
            Elevator.Set_AlgaeProcessor(525);
        }
        //#endregion

        // Elevator & Arm: Home
        //#region
        else if(driverController.getPOV() > 0){ //180 is down
            //Home pose
            Arm.Set_Home(0);
            Elevator.Set_Home(0);
        }
        //#endregion

        // Elevator & Arm: Stopped
        //#region
        if(driverController.getOptionsButton()){
            Arm.Stop();
            Elevator.Stop();
        }
        //#endregion

        // Switch with Coral scoring & Algea scoring
        //#region
        if(haspressed == false && driverController.getR1ButtonPressed()){
            haspressed = true;
        }else if(haspressed == true && driverController.getR1ButtonPressed()){
            haspressed = false;
        }
        //#endregion
    
        //----------------------------------------------------------------
        // Coral Cannon
        //----------------------------------------------------------------
        // - Holding L2 = Outtaking coral
        // - Holding R2 = Intaking coral 
        //----------------------------------------------------------------

        // Coral Cannon: Outtake
        //#region
        if(driverController.getR2Axis() > 0 && CoralInserted == true){
            // Automatisation after shooting out
            if (TimerTrigger == false){
                CoralTimer.reset();
                CoralTimer.start();
                TimerTrigger = true;
            }
            if(CoralTimer.get() > 1){
                CoralCannon.setPower(0);
                Arm.Set_Home(0);
                Elevator.Set_Home(0);
                
                CoralTimer.stop();
                TimerTrigger = true;
                CoralInserted = false;
            }
            // Spitting coral out
            else{
                CoralCannon.setPower(0.3);
            }
        //#endregion
        
        // Coral Cannon: Outtake
        //#region
        }else if(driverController.getL2Axis() > 0 && CoralInserted == false){
            
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
        }else{ 
            CoralCannon.setPower(0);
        }
        //#endregion
    
        //----------------------------------------------------------------
        // SmartDashboard
        //----------------------------------------------------------------
        //#region
        SmartDashboard.putString("LiftPose", Elevator.STS.LiftPosition);
        SmartDashboard.putNumber("LiftPoint", Elevator.Motor.STS.get_position());
        
        SmartDashboard.putString("ArmPose", Arm.STS.ArmPosition);
        SmartDashboard.putNumber("ArmPoint", Arm.Motor.STS.get_position());

        SmartDashboard.putBoolean("pressed", haspressed);

        //SmartDashboard.putBoolean("touch", touch1.get());
        //#endregion
        
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