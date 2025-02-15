package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import frc.robot.Library.NEO_PID;
import frc.robot.Library.NEO_SpeedCtrl;

//----------------------------------------------------------------
// Elevator Subsystem
//----------------------------------------------------------------
public class Elevator {
    //----------------------------------------------------------------
    // Objects
    //----------------------------------------------------------------
    private int elevatorCan_id = 23;
    private int FollowerCan_id = 24;

    private NEO_SpeedCtrl motorconfig = new NEO_SpeedCtrl(elevatorCan_id, true, false);
    public NEO_PID ElevatorMotor = new NEO_PID(motorconfig);
    private SparkMax followmotor = new SparkMax(FollowerCan_id, MotorType.kBrushless);
    private SparkMaxConfig configFollow = new SparkMaxConfig();

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
      private final double L1Position = 45;
      private final double L2Position = 90;
      private final double L3Position = 115;
      private final double L4Position = 180;
      private final double CorralStation = 120;
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
        ElevatorMotor.PAR.kP = 0.1;
        ElevatorMotor.PAR.kI = 1e-4;
        ElevatorMotor.PAR.kD = 1;
        ElevatorMotor.PAR.minOutputRange = -1;
        ElevatorMotor.PAR.maxOutputRange = 1;
        ElevatorMotor.PAR.maxVelocity = 500; //[RPM] was 4200
        ElevatorMotor.PAR.maxAcceleration = 250; //[RPM] was 6000
        ElevatorMotor.PAR.allowedClosedLoopError = 0.5;


        configFollow.follow(elevatorCan_id,true);
        
        followmotor.configure(configFollow, null,null);
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

    public void SetElevator(){

        switch(position) {
            case Zero: 
              ElevatorMotor.set_pos(PAR.ZeroPosition);
              break;

            case L1: 
              ElevatorMotor.set_pos(PAR.L1Position);
              break;

            case L2: 
              ElevatorMotor.set_pos(PAR.L2Position);
              break;

            case L3: 
              ElevatorMotor.set_pos(PAR.L3Position);
              break;

            case L4: 
              ElevatorMotor.set_pos(PAR.L4Position);
              break;

            case CorralStation: 
              ElevatorMotor.set_pos(PAR.CorralStation);
              break;

            default:
              
          }
    }
}
