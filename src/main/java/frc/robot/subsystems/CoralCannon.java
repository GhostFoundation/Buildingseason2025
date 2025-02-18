package frc.robot.subsystems;
import frc.robot.Library.*;

public class CoralCannon {
    
    public TalonFX_SpeedCtrl Motorcannon = new TalonFX_SpeedCtrl(1, true, false);   // MOTOR VAN CORAL CANNON HARDWAREMAP
    public TalonFX_SpeedCtrl MotorAlgea = new TalonFX_SpeedCtrl(2, true, false);    // MOTOR VAN ALGEA HARDWAREMAP

    public Digital_Input CoralDetecter = new Digital_Input(0, false); // SENSOR HWM 


    public double Launche() {

        Motorcannon.CS.SetSpeed = 0.5;  // Snelheid
        MotorAlgea.CS.SetSpeed = 0.5;   // Snelheid
        
        Motorcannon.STS.ActualSpeed();  // laat Snelheid zien op de station. rotation/min
        MotorAlgea.STS.ActualSpeed();   // laat Snelheid zien op de station. rotation/min

        return Motorcannon.STS.ActualSpeed();

    }

    /**
     * Constructor for the TalonFX Speed Controller
     * @param InTrigger The Can-id of the motor
     * @param Bumbperbutton  
     * @param Laun
e
     */
    public void CannonControll(Boolean L_Bumperbutton, Boolean R_BumperButton, boolean Laucher ) {   // nigger

            Motorcannon.StateActivation(L_Bumperbutton && CoralDetecter.STS.State() == false);   // in
            Motorcannon.StateActivation(R_BumperButton && CoralDetecter.STS.State() == true);   // uit
            
            MotorAlgea.StateActivation(Laucher);

    }
    

}
