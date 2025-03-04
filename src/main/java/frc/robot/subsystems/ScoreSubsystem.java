package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoreSubsystem extends SubsystemBase{
    public TalonFX Cannon = new TalonFX(40);
    
    

    public void setPower(double power){
        Cannon.setNeutralMode(NeutralModeValue.Brake);
        Cannon.set(power);
    }
}
