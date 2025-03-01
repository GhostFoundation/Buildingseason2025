package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ScoreSubsystem extends SubsystemBase{
    public TalonFX Cannon = new TalonFX(21);

    public void setPower(double power){
        Cannon.set(power);
    }

    /**
     * Mehtod to know if the corral is inserted in the Coral Cannon. 
     * @return Boolean state. When true the corral is inserted. When false the corral is not inserted.
     */
    public boolean CoralInPosition(){
        var StatorCurrent       = Cannon.getStatorCurrent();
        double GetStatorCurrent = StatorCurrent.getValueAsDouble();

        if(GetStatorCurrent > 10){
            return true;
        }
        else{
            return false;
        }
    }
}
