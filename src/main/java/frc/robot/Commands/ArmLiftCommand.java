// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ScoreSubsystem;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmLiftCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem Arm;
  private final ElevatorSubsystem Lift;
  
  public double ArmPose;
  public double LiftPose;
  public boolean Done = false;

  private final Timer timer = new Timer();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmLiftCommand(ArmSubsystem subsystem, double ArmPose, ElevatorSubsystem subsystem2, double LiftPose) {
    this.Arm = subsystem;
    this.Lift = subsystem2;
   


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(subsystem2);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  timer.reset();
  timer.start();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Arm.Setposition(ArmPose);
    Lift.Setposition(LiftPose);
}
   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.Setposition(0);
    Lift.Setposition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Done;    
  }


}
