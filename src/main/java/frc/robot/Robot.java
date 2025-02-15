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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.AprilTagVisionSubsystem;
import frc.robot.Library.*;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Coral;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private AprilTagVisionSubsystem m_visionSubsystem;
    private double ArmPower = 0.15;
    
    private boolean hasSeen = true;
    private double currentPoint;
    private double targetPoint;
    private double difference;
    // final DigitalInput sensor = new DigitalInput(0);

    private boolean hasPressed = false;

    private FakePS4Controller Operatorcontroller = new FakePS4Controller(1);
    private FakePS4Controller Drivercontroller = new FakePS4Controller(0);

    private SparkMax ElevatorMotor
     = new SparkMax(23, MotorType.kBrushless);
    private SparkMax followmotor = new SparkMax(24, MotorType.kBrushless);
    private SparkMaxConfig configFollow = new SparkMaxConfig();
    
    //arm motor
    private SparkMax ArmMotor = new SparkMax(25, MotorType.kBrushless);

    //coral cannon
    private TalonFX cc = new TalonFX(21);

    @Override
    public void robotInit() {
       m_robotContainer = new RobotContainer();
        String leftCameraName = "leftCam";
        String rightCameraName = "rightCam";
        Transform3d cameraToRobot = new Transform3d(new Translation3d(0.15, 0, 0), new Rotation3d(0, 0, 0));
        m_visionSubsystem = new AprilTagVisionSubsystem(leftCameraName, rightCameraName, cameraToRobot);
        
        configFollow.follow(23,true);
        followmotor.configure(configFollow, null, null);

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
        
    }

    @Override
    public void teleopPeriodic() {
        //----------------------------------------------------------------
        // Elevator
        //----------------------------------------------------------------
        // Connect controller buttons with positions
        if(Drivercontroller.getCircleButton()){
            //move the elevator upwards
            ElevatorMotor.set(0.6);
            // followmotor.set(1);
        }else if(Drivercontroller.getSquareButton()){
            //move the elevator downwards
            ElevatorMotor.set(-0.2);
            // followmotor.set(-1);
        }else{
            ElevatorMotor.set(0);
            // followmotor.set(0);
        }

        if(Drivercontroller.getCrossButton()){
            ArmMotor.set(ArmPower);
        }else if(Drivercontroller.getTriangleButton()){
            ArmMotor.set(-ArmPower);
        }else{
            ArmMotor.set(0.0);
        }

        if(Drivercontroller.getR1Button()){
            cc.set(0.3);
        }else if (Drivercontroller.getL1Button()) {
            cc.set(-0.2);
        }else{
            cc.set(0);
        }




        // if(Operatorcontroller.getCircleButton()){
        //     targetPoint = 1;
        // }else if(Operatorcontroller.getTriangleButton()){
        //     targetPoint = 2;
        // }else if(Operatorcontroller.getSquareButton()){
        //     targetPoint = 3;
        // }else if(Operatorcontroller.getCrossButton()){
        //     targetPoint = 0;
        // }

        // //the sensor sees the plate and it hasnt yet seen a plate
        // if(sensor.get() == false && hasSeen == false){
        //     SmartDashboard.putString("sensor", "true");
        //    //once it sees one of the plates
        //    if(difference < 0){
        //    currentPoint++;
        //    hasSeen = true;
        //    }else if(difference > 0){
        //     currentPoint--;
        //     hasSeen = true;
        //    }
        // }else if(sensor.get() == true){
        //     SmartDashboard.putString("sensor","false");
        //     //once its past one of the plates
        //     hasSeen = false;   
        // }

        // difference = currentPoint - targetPoint;
        // if(difference > 0){
        //     //set the power to negative / the lift moves down
        //     ElevatorMotor.set(-0.2);
        //     SmartDashboard.putString("lift","going down");
        // }else if(difference < 0){
        //     //set the power to positive / the lift moves up
        //     ElevatorMotor.set(0.4);
        //     SmartDashboard.putString("lift","going up");
        // }else if(difference ==0){
        //     //set the power to 0 / the lift stops moving
        //     ElevatorMotor.set(0);
        //     SmartDashboard.putString("lift","stopping");
        // }

        // SmartDashboard.putNumber("target", targetPoint);
        // SmartDashboard.putNumber( "current", currentPoint);
        // SmartDashboard.putBoolean("hasSeen", hasSeen);
        // SmartDashboard.putBoolean("sensor", sensor.get());
        
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