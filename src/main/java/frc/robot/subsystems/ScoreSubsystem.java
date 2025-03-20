package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library.Digital_Input;

public class ScoreSubsystem extends SubsystemBase{
    public TalonFX Cannon = new TalonFX(40);
    final Digital_Input sensor = new Digital_Input(2, false); //
    final Digital_Input sensor2 = new Digital_Input(3, false); //
     
    public void setPower(double power){
        Cannon.setNeutralMode(NeutralModeValue.Brake);
        Cannon.set(power);
    }

    public boolean CoralInside(){
        if(sensor2.STS.State()){
            return true;
        }
        else{
            return false;
        }
    }
    public boolean CoralInpose(){
        if(sensor2.STS.State() == false && sensor.STS.State()){
            return true;
        }else{
            return false;
        }
    }
    public double Current(){
        return Cannon.getStatorCurrent().getValueAsDouble();
    }
}
