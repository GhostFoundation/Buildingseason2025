package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoreSubsystem extends SubsystemBase{
    public TalonFX Cannon = new TalonFX(40);
    final DigitalInput sensor = new DigitalInput(2);
    

    public void setPower(double power){
        Cannon.setNeutralMode(NeutralModeValue.Brake);
        Cannon.set(power);
    }

    public boolean CoralInPosition(){
        var StatorCurrent       = Cannon.getStatorCurrent();
        double GetStatorCurrent = StatorCurrent.getValueAsDouble();

        if(GetStatorCurrent > 50 && sensor.get() == false){
            return true;
        }
        else{
            return false;
        }
    }
    public double Current(){
        return Cannon.getStatorCurrent().getValueAsDouble();
    }
}
