package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import frc.robot.Library.NEO_PID;
import frc.robot.Library.NEO_SpeedCtrl;
import frc.robot.Library.TalonFX_SpeedCtrl;

//----------------------------------------------------------------
// Coral Subsystem
//----------------------------------------------------------------
public class Coral {
    //----------------------------------------------------------------
    // Objects
    //----------------------------------------------------------------
    private int coral_id = 21;

    private TalonFX_SpeedCtrl coralMotor = new TalonFX_SpeedCtrl(coral_id, true, false);
    
    public sts STS = new sts();

    private int State;
    //----------------------------------------------------------------
    // Variables
    //----------------------------------------------------------------

    public class sts {
        private boolean InTrigger = false;
        public boolean InTrigger(){
          return InTrigger;
        }
        private boolean OutTrigger = false;
        public boolean OutTrigger(){
            return OutTrigger;
        }
      }
    
   

     
    
    //----------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------
    public void init(){

    }
    public void SetIn(Boolean Trigger){
      STS.InTrigger = Trigger;
      if(STS.InTrigger){
        State = 1;
      }
    }
    public void SetOut(Boolean Trigger){
        STS.OutTrigger = Trigger;
        if(STS.OutTrigger){
          coralMotor.CS.SetSpeed = 0.5;
        }
    }

    
    public void SetCC(){

        switch(State) {
            case 1: 
                coralMotor.CS.SetSpeed = 0.5;
              break;

            case 2: 
                coralMotor.CS.SetSpeed = -0.5;
              break;

            default:
            coralMotor.CS.SetSpeed = 0;
          }
    
    }
}
