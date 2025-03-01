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
        public class sts{
                public String ArmPosition;
        }

        // Parameter
        public class par{
                private final double ArmConvertion = 0.077777778;
        }

        //----------------------------------------------------------------
        // Control Modules
        //----------------------------------------------------------------
        private SparkMax motor = new SparkMax(30, MotorType.kBrushless);
        public NEO_Relative_PID Motor = new NEO_Relative_PID(motor);

        //----------------------------------------------------------------
        // Constructor
        //----------------------------------------------------------------
        public ArmSubsystem(){
                Motor.set_P(1);//1
                Motor.set_I(0);//0
                Motor.set_D(0);//0
                Motor.set_allowedClosedLoopError(0.075);//0.075
                Motor.set_maxVelocity(2000);//500
                Motor.set_maxAcceleration(4000);//2500
        }

        //----------------------------------------------------------------
        // Methods
        //----------------------------------------------------------------
        public void Zero(){
                Motor.SetZero();
        }

        public void Setposition(double pos_degrees){
                Motor.Set_position(pos_degrees * 0.077777778);
        }

        public void Set_Home(double height){
                Motor.Set_position(height * PAR.ArmConvertion);
                STS.ArmPosition = "Home";
        }

        public void Set_L1(double height){
                Motor.Set_position(height * PAR.ArmConvertion);
                STS.ArmPosition = "L1";
        }

        public void Set_L2(double height){
                Motor.Set_position(height * PAR.ArmConvertion);
                STS.ArmPosition = "L2";
        }

        public void Set_L3(double height){
                Motor.Set_position(height * PAR.ArmConvertion);
                STS.ArmPosition = "L3";
        }

        public void Set_L4(double height){
                Motor.Set_position(height * PAR.ArmConvertion);
                STS.ArmPosition = "L4";
        }

        public void Set_CoralStation(double height){
                Motor.Set_position(height * PAR.ArmConvertion);
                STS.ArmPosition = "Coral Station";
        }

        public void Set_AlgaeLow(double height){
                Motor.Set_position(height * PAR.ArmConvertion);
                STS.ArmPosition = "Algae Low";
        }

        public void Set_AlgaeHigh(double height){
                Motor.Set_position(height * PAR.ArmConvertion);
                STS.ArmPosition = "Algae High";
        }

        public void Set_AlgaeNet(double height){
                Motor.Set_position(height * PAR.ArmConvertion);
                STS.ArmPosition = "Algae Net";
        }

        public void Set_AlgaeProcessor(double height){
                Motor.Set_position(height * PAR.ArmConvertion);
                STS.ArmPosition = "Algae Processor";
        }

        public void Stop(){
                motor.stopMotor();
        }


        

}
