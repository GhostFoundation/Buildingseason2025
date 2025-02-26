package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoreSubsystem extends SubsystemBase{
    public TalonFX Cannon = new TalonFX(21);

    public void setPower(double power){
        Cannon.set(power);
    }
}
