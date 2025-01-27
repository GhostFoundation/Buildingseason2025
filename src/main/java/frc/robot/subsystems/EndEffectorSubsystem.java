// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class EndEffectorSubsystem extends SubsystemBase {
  private final TalonFX motor1 = new TalonFX(21);
  private final TalonFX motor2 = new TalonFX(22);

  

  public EndEffectorSubsystem() {

  }
  public void setMotorRight(){
    motor1.set(1);
    motor2.set(1);
  }
  public void setMotorLeft(){
    motor1.set(-1);
    motor2.set(-1);
  }


  public void StopMotor(){
    motor1.stopMotor();
    motor2.stopMotor();
  }

}
