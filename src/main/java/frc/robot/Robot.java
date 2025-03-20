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
    // Subsystems objects
    private ElevatorSubsystem Lift;
    private ArmSubsystem Arm;
    private ScoreSubsystem CoralCannon;

    private AprilTagVisionSubsystem m_visionSubsystem;

    // Timer object
    private final Timer CoralTimer = new Timer();
   
    //LEDs
    private Leds Ledstrip = new Leds();
    //sensors
    //final DigitalInput touch1 = new DigitalInput(0);
    //final DigitalInput touch2 = new DigitalInput(1);
    // final DigitalInput sensor = new DigitalInput(2);

    private FakePS4Controller driverController = new FakePS4Controller(0);
    private FakePS4Controller operatorController = new FakePS4Controller(1);

    static CameraServer TunnelVision;

    //Arm subsystem
    String ArmPosition = "Zero";

    //Elevator subsystem;
    String LiftPosition = "Zero";
    double LiftSetpoint = 0;
    boolean CorrectionRunOnce = false;
    double ArmSetpoint = 0;
    boolean hasTouched = false;

    //pressed
    boolean haspressed = false;
    boolean playerss = false;

    // CoralCanon
    boolean CoralInside = false;
    boolean CoralInpose = false;
    boolean TimerTrigger = false;
    boolean ShotOut = false;
    double IntakePower = 0;
   
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        Lift = m_robotContainer.Lift;
        Arm = m_robotContainer.Arm;
        CoralCannon = m_robotContainer.cc;

        String leftCameraName = "leftCam";
        String rightCameraName = "rightCam";
        Transform3d cameraToRobot = new Transform3d(new Translation3d(0.15, 0, 0), new Rotation3d(0, 0, 0));
        m_visionSubsystem = new AprilTagVisionSubsystem(leftCameraName, rightCameraName, cameraToRobot);
        
        CameraServer.startAutomaticCapture();
        
        //Initialising the arm
        Arm.Armmotor.SetZero();
        // initialising the elevator
        Lift.ElevatorMotor.SetZero();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        CoralInside = CoralCannon.CoralInside();
        CoralInpose = CoralCannon.CoralInpose();
        /*
        try {
            Pose3d robotPose = m_visionSubsystem.calculateRobotPose();
            //SmartDashboard.putString("robot pose according to apriltags", robotPose.toString());
        } catch (Exception e) {
            //
            SmartDashboard.putString("apriltag error", e.getMessage());
        }
            */
        // TODO Add end switch to elevator subsystem
        // Lift End Switch logic
        
        // if(hasTouched == false && touch1.get()==false && touch2.get() ==false){
        //     Lift.Stop();
        //     Lift.Zero();
        //     LiftSetpoint = 0;
        //     hasTouched = true;
        // }else if(hasTouched == true && (touch1.get() == true || touch2.get() == true)){
        //     hasTouched = false;
        // }
        
        
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
        double Aantal_Keer_LED_Knipperen =  5;
        double LED_Knipper_Interval = 0.5; // is in seconden; 1=
                                       // 1sec aan en 1 sec
                                       // uit; 0,5 is half sec
                                       // aan en 0,5 sec uit
                                       // etc

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

        // Switching mode : Algae/Coral
        //#region
        if(haspressed == false && driverController.getR1ButtonPressed()){
            haspressed = true;
            Ledstrip.Led_Strip_Solid(0, 255, 255);
        }else if(haspressed == true && driverController.getR1ButtonPressed()){
            haspressed = false;
            Ledstrip.Led_Strip_Solid(160, 32, 240);
        }
        //#endregion

        // Coral Mode: L1 Position
        //#region
        if (operatorController.getCrossButton() && haspressed == false){
            LiftSetpoint = 0;
            ArmSetpoint = 0; // -40
            
            LiftPosition = "L1";
            ArmPosition = "L1Scoring";
        }
        //#endregion

        // Coral Mode: L2 Position
        //#region
        else if(operatorController.getSquareButton() && haspressed == false){
            LiftSetpoint = 200; //0
            ArmSetpoint = 0; // 150

            LiftPosition = "L2";
            ArmPosition = "L2Scoring";
        }
        //#endregion

        // Coral Mode: L3 Position
        //#region
        else if(operatorController.getTriangleButton() && haspressed == false){
            LiftSetpoint = 400; //170
            ArmSetpoint = 0; // 160

            LiftPosition = "L3";
            ArmPosition = "L3Scoring";
        }
        //#endregion

        // Coral Mode: L4 Position
        //#region
        else if(operatorController.getCircleButton() && haspressed == false){ 
            LiftSetpoint = 480;
            ArmSetpoint = 0; // 170

            LiftPosition = "L4";
            ArmPosition = "L4Scoring";
        }
        //#endregion
        
        // Algae Mode: Low position
        //#region
        else if (operatorController.getSquareButton() && haspressed == true){
            LiftSetpoint = 75;
            ArmSetpoint = 20; //70
            
            LiftPosition = "Algae low";
            ArmPosition = "Algae";
        }
        //#endregion

        // Algae Mode: High position
        //#region
        else if(operatorController.getTriangleButton() && haspressed == true){
            LiftSetpoint = 300;
            ArmSetpoint = 20; // 70

            LiftPosition = "Algae high";
            ArmPosition = "Algae";
        }
        //#endregion

        // Home position
        //#region
        else if(operatorController.getPOV() > 0){ //180 is down
            LiftSetpoint = 0;
            ArmSetpoint = 0;

            LiftPosition = "Home";
            ArmPosition = "Home";
        }
        //#endregion

        // Coral Station position
        //#region
        else if(operatorController.getL1Button()){
            LiftSetpoint = 350; //312
            ArmSetpoint = 0; // -15
            IntakePower = 0.3;
            LiftPosition = "Coral Station";
            ArmPosition = "Intake";
            Ledstrip.Led_Strip_Knipper(160, 32, 240, Aantal_Keer_LED_Knipperen, LED_Knipper_Interval);
        }
        //#endregion

        // Stop Elevator & Arm
        //#region
        if(driverController.getOptionsButton()){
            m_robotContainer.Arm.Stop();
            Lift.Stop();
            IntakePower = 0;
        }
        //#endregion

        // Manual Lift Corection
        //#region
        if((operatorController.getR2Axis() > 0) && CorrectionRunOnce == false){
            LiftSetpoint = LiftSetpoint + 5;
            CorrectionRunOnce = true;
        }
        else if((operatorController.getL2Axis() > 0) && CorrectionRunOnce == false){
            LiftSetpoint = LiftSetpoint - 5;
            CorrectionRunOnce = true;
        }
        else if((operatorController.getL2Axis() == 0) && (operatorController.getR2Axis() == 0) && CorrectionRunOnce == true){
            CorrectionRunOnce = false;
        }
        //#endregion

        // Activation Lift & Arm
        //#region
        // Activation Lift
        Lift.Setposition(LiftSetpoint);
        // Activation Arm
        Arm.Setposition(ArmSetpoint);
        // Activation Cannon
        CoralCannon.setPower(IntakePower);
        //#endregion
    
        //----------------------------------------------------------------
        // Coral Cannon
        // Holding R2 = Intaking coral
        // Holding L2 = Outtaking coral
        //----------------------------------------------------------------

        // After shooting trigger after 1 sec the coral is not inside end-effector
        //#region
        if(CoralTimer.get() > 1.5 && ShotOut == true){
            LiftSetpoint = 0;
            CoralInpose = false;
            CoralInside = false;
            ShotOut = false;
        }
        //#endregion

        //
        //#region
        if(CoralInpose){
            IntakePower = 0;
            LiftSetpoint = 0;
            ShotOut = false;
            Ledstrip.Led_Strip_Knipper(255, 0, 0, Aantal_Keer_LED_Knipperen, LED_Knipper_Interval);
        }
        if(CoralInside){
            LiftSetpoint = 0;
            IntakePower = 0.1;
            ShotOut = false;
        }
        //#endregion


        // Coral Cannon Shooting Coral
        //#region
        if(driverController.getR2Axis() > 0 && CoralInpose == true && haspressed == false){
            // Start timer
            if (TimerTrigger == false){
                CoralTimer.reset();
                CoralTimer.start();
                TimerTrigger = true;
            }
            // outtaking coral
            IntakePower = 0.3;
            ShotOut = true;
        }
        //#endregion


        //Coral Cannon: Algae mode
        //#region
        if(driverController.getR2Axis() > 0 && haspressed == true){
            IntakePower = 0;
        }else if(driverController.getL2Axis() > 0 && haspressed == true){
            IntakePower = -0.5;
        }else if(haspressed == true){
            IntakePower = 0;
        }
        //#endregion
        
        //----------------------------------------------------------------
        // SmartDashboard
        //----------------------------------------------------------------
        //#region
        SmartDashboard.putBoolean("InPosition", CoralInpose);
        SmartDashboard.putBoolean("Inside", CoralInside);
        SmartDashboard.putNumber("Timer", CoralTimer.get());
        SmartDashboard.putNumber("lift setpoint", LiftSetpoint);
        SmartDashboard.putBoolean("Mode", haspressed);

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