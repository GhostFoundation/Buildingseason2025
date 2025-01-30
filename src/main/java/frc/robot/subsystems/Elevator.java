package frc.robot.subsystems;

import frc.robot.Library.NEO_PID;
import frc.robot.Library.NEO_SpeedCtrl;

//----------------------------------------------------------------
// Elevator Subsystem
//----------------------------------------------------------------
public class Elevator {
    //----------------------------------------------------------------
    // Objects
    //----------------------------------------------------------------
    private NEO_SpeedCtrl motorconfig = new NEO_SpeedCtrl(0, true, false);
    public NEO_PID ElevatorMotor = new NEO_PID(motorconfig);

    public Positions positions = new Positions();

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

    enum Position {
        Zero,
        L1,
        L2,
        L3,
        L4,
        CorralStation
    }

    

    
    //----------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------
    public void init(){
        ElevatorMotor.PAR.kP = 1;
        ElevatorMotor.PAR.kI = 0;
        ElevatorMotor.PAR.kD = 0;

        ElevatorMotor.setconfig();
    }

    public void SetPosition(int Setposition){
        Position position = Position.values()[Setposition];
        switch(position) {
            case Zero: 
              ElevatorMotor.set_pos(0);
              break;

            case L1: 
              ElevatorMotor.set_pos(720);
              break;

            case L2: 
              
              break;

            case L3: 
              
              break;

            case L4: 
              
              break;

            case CorralStation: 
              
              break;

            default:
              
          }
    }
}
