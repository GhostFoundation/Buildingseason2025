package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Library.*;

public class ArmSubsystem extends SubsystemBase{
        //----------------------------------------------------------------
        // Attributes
        //----------------------------------------------------------------
        public sts STS = new sts();
        public cs CS = new cs();
        public par PAR = new par();

        // Control Signal
        public class cs{}

        // Status
        public class sts{}

        // Parameter
        public class par{}

        //----------------------------------------------------------------
        // Control Modules
        //----------------------------------------------------------------
        private SparkMax motor = new SparkMax(30, MotorType.kBrushless);
        public NEO_Relative_PID Armmotor = new NEO_Relative_PID(motor);

        //----------------------------------------------------------------
        // Constructor
        //----------------------------------------------------------------
        public ArmSubsystem(){
                Armmotor.set_P(1);//1
                Armmotor.set_I(0);//0
                Armmotor.set_D(0);//0
                Armmotor.set_allowedClosedLoopError(0.075);//0.075
                Armmotor.set_maxVelocity(2000);//500
                Armmotor.set_maxAcceleration(4000);//2500
        }

        //----------------------------------------------------------------
        // Methods
        //----------------------------------------------------------------
        public void Zero(){
                Armmotor.SetZero();
        }

        public void Setposition(double pos_degrees){
                Armmotor.Set_position(pos_degrees * 0.077777778);
        }

        public void Stop(){
                motor.stopMotor();
        }


        

}
