package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Library.NEO_Relative_PID;

//----------------------------------------------------------------
// Arm Subsystem
//----------------------------------------------------------------
public class ArmSubsystem {
    //----------------------------------------------------------------
    // Objects
    //----------------------------------------------------------------

    private int Arm_CanID = 3;

    private NEO_Relative_PID ArmMotor = new NEO_Relative_PID(new SparkMax(Arm_CanID, MotorType.kBrushless));
    private Angles angles = new Angles();
    public sts STS = new sts();
    public par PAR = new par();

    //----------------------------------------------------------------
    // Variables
    //----------------------------------------------------------------
    public class Angles {
        public final int Zero = 0;
        public final int A1 = 1;
        public final int A2 = 2;
        public final int A3 = 3;
        public final int A4 = 4;
        public final int A5 = 5;
    }

    public class sts {
        private boolean ZeroTrigger = false;
        private boolean A1Trigger = false;
        private boolean A2Trigger = false;
        private boolean A3Trigger = false;
        private boolean A4Trigger = false;
        private boolean A5Trigger = false;

        public boolean ZeroTrigger() {
            return ZeroTrigger;
        }

        public boolean A1Trigger() {
            return A1Trigger;
        }

        public boolean A2Trigger() {
            return A2Trigger;
        }

        public boolean A3Trigger() {
            return A3Trigger;
        }

        public boolean A4Trigger() {
            return A4Trigger;
        }

        public boolean A5Trigger() {
            return A5Trigger;
        }
    }

    public class par {
        //Replace with real values! Test the values since it is relative in terms of rev
        private final double ZeroPosition = 0; //[rotations]
        private double A1Position = 1;
        private double A2Position = 2;
        private double A3Position = 3;
        private double A4Position = 4;
        private double A5Position = 5;
    }

    enum Angle {
        Zero,
        A1,
        A2,
        A3,
        A4,
        A5
    }

    Angle TargetAngle = Angle.values()[angles.Zero]; //since angles.zero is 0, it accesses the 0th index of enum, and assign that value to TargetAngle

    //----------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------
    public void init() {
        ArmMotor.PAR.P_Gain = 1;
        ArmMotor.PAR.I_Gain = 0;
        ArmMotor.PAR.D_Gain = 0;
        ArmMotor.PAR.FF_Gain = 0;
        ArmMotor.PAR.Output_Min = -1;
        ArmMotor.PAR.Output_Max = 1;
        ArmMotor.PAR.maxVelocity = 4200; //[RPM]
        ArmMotor.PAR.maxAcceleration = 6000; //[RPM]
        ArmMotor.PAR.allowedClosedLoopError = 0.5;
        ArmMotor.setconfig();
    }

    public void SetZeroPosition(boolean Trigger){
        STS.ZeroTrigger = Trigger;
        if (STS.ZeroTrigger) {
            TargetAngle = Angle.values()[angles.Zero];
        }
    }

    public void SetA1(boolean Trigger){
        STS.A1Trigger = Trigger;
        if (STS.A1Trigger) {
            TargetAngle = Angle.values()[angles.A1];
        }
    }

    public void SetA2(boolean Trigger){
        STS.A2Trigger = Trigger;
        if (STS.A2Trigger) {
            TargetAngle = Angle.values()[angles.A2];
        }
    }

    public void SetA3(boolean Trigger){
        STS.A3Trigger = Trigger;
        if (STS.A3Trigger) {
            TargetAngle = Angle.values()[angles.A3];
        }
    }

    public void SetA4(boolean Trigger){
        STS.A4Trigger = Trigger;
        if (STS.A4Trigger) {
            TargetAngle = Angle.values()[angles.A4];
        }
    }

    public void SetA5(boolean Trigger){
        STS.A5Trigger = Trigger;
        if (STS.A5Trigger) {
            TargetAngle = Angle.values()[angles.A5];
        }
    }

    //Case to actully set the angle of the motor
    public void setAngle(){
        switch (TargetAngle) {
            case Zero:
                ArmMotor.Set_position(PAR.ZeroPosition);
                break;
            case A1:
                ArmMotor.Set_position(PAR.A1Position);
                break;
            case A2:
                ArmMotor.Set_position(PAR.A2Position);
                break;
            case A3:
                ArmMotor.Set_position(PAR.A3Position);
                break;
            case A4:
                ArmMotor.Set_position(PAR.A4Position);
                break;
            case A5:
                ArmMotor.Set_position(PAR.A5Position);
                break;
        }
    }
}
