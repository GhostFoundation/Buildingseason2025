package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import frc.robot.Library.NEO_PID;
import frc.robot.Library.NEO_SpeedCtrl;

//----------------------------------------------------------------
// Arm Subsystem
//----------------------------------------------------------------
public class Arm {
    //----------------------------------------------------------------
    // Objects
    //----------------------------------------------------------------
    private int arm_id = 25;

    private NEO_SpeedCtrl motorconfig = new NEO_SpeedCtrl(arm_id, true, false);
    public NEO_PID ArmMotor = new NEO_PID(motorconfig);

    public Positions positions = new Positions();
    public sts STS = new sts();
    public par PAR = new par();
    //----------------------------------------------------------------
    // Variables
    //----------------------------------------------------------------
    public class Positions {
        public final int Zero = 0;
        public final int L1 = 1;
        public final int L2 = 2;
        public final int L3 = 3;
        public final int L4 = 4;
        public final int CorralStation = 5;
      }

    public class sts {
        private boolean ZeroTrigger = false;
        private boolean L1Trigger = false;
        private boolean L2Trigger = false;
        private boolean L3Trigger = false;
        private boolean L4Trigger = false;
        private boolean CorralStationTrigger = false;

        public boolean ZeroTrigger(){
          return ZeroTrigger;
        }

        public boolean L1Trigger(){
          return L1Trigger;
        }

        public boolean L2Trigger(){
          return L2Trigger;
        }

        public boolean L3Trigger(){
          return L3Trigger;
        }

        public boolean L4Trigger(){
          return L4Trigger;
        }

        public boolean CorralStationTrigger(){
          return CorralStationTrigger;
        }
      }
    
    public class par{
      private final double ZeroPosition = 0; //[degrees]
      private final double L1Position = -40;
      private final double L2Position = 140;
      private final double L3Position = 140;
      private final double L4Position = 142.5;
      private final double CorralStation = -27;
    }

    enum Position {
        Zero,
        L1,
        L2,
        L3,
        L4,
        CorralStation
    }

      Position position = Position.values()[positions.Zero];
    
    //----------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------
    public void init(){
        ArmMotor.PAR.kP = 0.1;
        ArmMotor.PAR.kI = 0;
        ArmMotor.PAR.kD = 0;
        ArmMotor.PAR.minOutputRange = -1;
        ArmMotor.PAR.maxOutputRange = 1;
        ArmMotor.PAR.maxVelocity = 4200; //[RPM]
        ArmMotor.PAR.maxAcceleration = 6000; //[RPM]
        ArmMotor.PAR.allowedClosedLoopError = 0.5;



    }

    public void SetZeroPosition(Boolean Trigger){
      STS.ZeroTrigger = Trigger;
      if(STS.ZeroTrigger){
        position = Position.values()[positions.Zero];
      }
    }

    public void SetL1(Boolean Trigger){
      STS.L1Trigger = Trigger;
      if(STS.L1Trigger){
        position = Position.values()[positions.L1];
      }
    }

    public void SetL2(Boolean Trigger){
      STS.L2Trigger = Trigger;
      if(STS.L2Trigger){
        position = Position.values()[positions.L2];
      }
    }

    public void SetL3(Boolean Trigger){
      STS.L3Trigger = Trigger;
      if(STS.L3Trigger){
        position = Position.values()[positions.L3];
      }
    }

    public void SetL4(Boolean Trigger){
      STS.L4Trigger = Trigger;
      if(STS.L4Trigger){
        position = Position.values()[positions.L4];
      }
    }

    public void SetCorralStation(Boolean Trigger){
      STS.CorralStationTrigger = Trigger;
      if(STS.CorralStationTrigger){
        position = Position.values()[positions.CorralStation];
      }
    }

    public void SetArm(){

        switch(position) {
            case Zero: 
              ArmMotor.set_pos(PAR.ZeroPosition);
              break;

            case L1: 
              ArmMotor.set_pos(PAR.L1Position);
              break;

            case L2: 
              ArmMotor.set_pos(PAR.L2Position);
              break;

            case L3: 
              ArmMotor.set_pos(PAR.L3Position);
              break;

            case L4: 
              ArmMotor.set_pos(PAR.L4Position);
              break;

            case CorralStation: 
              ArmMotor.set_pos(PAR.CorralStation);
              break;

            default:
              
          }
    }
}
