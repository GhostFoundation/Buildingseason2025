// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import frc.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ArmCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem Arm;
  private final double Pose;
  private final Timer timer = new Timer();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(ArmSubsystem subsystem, double pose) {
    this.Arm = subsystem;
    this.Pose = pose;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

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
    Arm.Setposition(Pose);
}
   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 1;    
  }
}
