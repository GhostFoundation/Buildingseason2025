// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.FullScoringCommand;
import frc.robot.Commands.ScoreCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Library.*;
import frc.robot.Library.FakePS4Controller.Button;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ScoreSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
      private final SendableChooser<Command> autoChooser;

    

    private final Field2d field;
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ArmSubsystem Arm = new ArmSubsystem();
  public final ElevatorSubsystem Lift = new ElevatorSubsystem();
  public final ScoreSubsystem cc = new ScoreSubsystem();
  
  // The driver's controller
  FakePS4Controller m_driverController = new FakePS4Controller(OIConstants.kDriverControllerPort);
  FakePS4Controller m_operatorController = new FakePS4Controller(OIConstants.kDriverControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    //TODO check if these simple commands can run in autonomous
    NamedCommands.registerCommand("ARM", new ArmCommand(Arm));
    // NamedCommands.registerCommand("LIFT", new ElevatorCommand(Lift, 200));
    NamedCommands.registerCommand("Score", new ScoreCommand(cc));
    // //TODO check if these combined commands work in autonomous
    //NamedCommands.registerCommand("ArmLiftHome", new ArmLiftCommand(Arm,Lift,"Home"));
    // NamedCommands.registerCommand("ArmLiftInt", new ArmLiftCommand(Arm,Lift,"Intake"));
    // NamedCommands.registerCommand("ArmLift1", new ArmLiftCommand(Arm,Lift,"L1"));
    NamedCommands.registerCommand("ArmLift2", new FullScoringCommand(Arm,144,Lift,0,cc));
    // NamedCommands.registerCommand("ArmLift3", new ArmLiftCommand(Arm,Lift,"L3"));
    // NamedCommands.registerCommand("ArmLift4", new ArmLiftCommand(Arm,Lift,"L4"));

    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));


        // Logging for pathplanner for PID tuning
    field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("PIDstraight").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("PIDstraight").setPoses(poses);
        });


    //auto selector:
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //new buttons
    // new JoystickButton(m_driverController, Button.kCross.value).onTrue(m_coral.setSetpointCommand(Setpoint.kRest));
    // new JoystickButton(m_driverController,Button.kTriangle.value).onTrue(m_coral.setSetpointCommand(Setpoint.kLevel2));

    //existing buttons
    new JoystickButton(m_driverController, Button.kShare.value).whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    //TODO check if this command actually works and does as told
    // new JoystickButton(m_driverController, Button.kCross.value).whileTrue(new ArmLiftCommand(Arm, Lift, "L1"));
    // new JoystickButton(m_driverController, Button.kSquare.value).whileTrue(new ArmLiftCommand(Arm, Lift, "L2"));
    // new JoystickButton(m_driverController, Button.kTriangle.value).whileTrue(new ArmLiftCommand(Arm, Lift, "L3"));
    // new JoystickButton(m_driverController, Button.kCircle.value).whileTrue(new ArmLiftCommand(Arm, Lift, "L4")); 
    // new JoystickButton(m_driverController, m_driverController.getPOV(0)).whileTrue(new ArmLiftCommand(Arm, Lift, "Home")); //TODO check if the pov works
    // new JoystickButton(m_driverController, m_driverController.getPOV(180)).whileTrue(new ArmLiftCommand(Arm, Lift, "Intake")); // TODO check if the pov works
    // new JoystickButton(m_driverController, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
