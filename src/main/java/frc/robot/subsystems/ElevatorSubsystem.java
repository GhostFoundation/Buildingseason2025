package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library.*;

public class ElevatorSubsystem extends SubsystemBase{
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
                public String LiftPosition;
        }

        // Parameter
        public class par{
                private final double ElevatorConvertion = 0.147058824;
        }

        //----------------------------------------------------------------
        // Control Modules
        //----------------------------------------------------------------
        private SparkMax motor = new SparkMax(20, MotorType.kBrushless);
        private SparkMax followMotor = new SparkMax(22,MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        
        public NEO_Relative_PID Motor = new NEO_Relative_PID(motor);
        
        //----------------------------------------------------------------
        // Constructor
        //----------------------------------------------------------------
        public ElevatorSubsystem(){
                config.follow(20,true);
                followMotor.configure(config, null, null);

                Motor.set_P(1);//1
                Motor.set_I(0);//0
                Motor.set_D(0);//0
                Motor.set_allowedClosedLoopError(0.1);//0.15
                Motor.set_maxVelocity(2500);//2500
                Motor.set_maxAcceleration(1500*3);// 2500*5
        }

        //----------------------------------------------------------------
        // Methods
        //----------------------------------------------------------------
        public void Zero(){
                Motor.SetZero();
        }

        public void Setposition(double height){
                Motor.Set_position(height * 0.147058824);
        }

        public void Set_Home(double height){
                Motor.Set_position(height * PAR.ElevatorConvertion);
                STS.LiftPosition = "Home";
        }

        public void Set_L1(double height){
                Motor.Set_position(height * PAR.ElevatorConvertion);
                STS.LiftPosition = "L1";
        }

        public void Set_L2(double height){
                Motor.Set_position(height * PAR.ElevatorConvertion);
                STS.LiftPosition = "L2";
        }

        public void Set_L3(double height){
                Motor.Set_position(height * PAR.ElevatorConvertion);
                STS.LiftPosition = "L3";
        }

        public void Set_L4(double height){
                Motor.Set_position(height * PAR.ElevatorConvertion);
                STS.LiftPosition = "L4";
        }

        public void Set_CoralStation(double height){
                Motor.Set_position(height * PAR.ElevatorConvertion);
                STS.LiftPosition = "Coral Station";
        }

        public void Set_AlgaeLow(double height){
                Motor.Set_position(height * PAR.ElevatorConvertion);
                STS.LiftPosition = "Algae Low";
        }

        public void Set_AlgaeHigh(double height){
                Motor.Set_position(height * PAR.ElevatorConvertion);
                STS.LiftPosition = "Algae High";
        }

        public void Set_AlgaeNet(double height){
                Motor.Set_position(height * PAR.ElevatorConvertion);
                STS.LiftPosition = "Algae Net";
        }

        public void Set_AlgaeProcessor(double height){
                Motor.Set_position(height * PAR.ElevatorConvertion);
                STS.LiftPosition = "Algae Processor";
        }

        public void Stop(){
                motor.stopMotor();
        }

}
