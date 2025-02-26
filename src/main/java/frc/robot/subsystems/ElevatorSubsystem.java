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
        public class sts{}

        // Parameter
        public class par{}

        //----------------------------------------------------------------
        // Control Modules
        //----------------------------------------------------------------
        private SparkMax motor = new SparkMax(20, MotorType.kBrushless);
        private SparkMax followMotor = new SparkMax(22,MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        
        public NEO_Relative_PID ElevatorMotor = new NEO_Relative_PID(motor);
        
        //----------------------------------------------------------------
        // Constructor
        //----------------------------------------------------------------
        public ElevatorSubsystem(){
                config.follow(20,true);
                followMotor.configure(config, null, null);

                ElevatorMotor.set_P(1);//1
                ElevatorMotor.set_I(0);//0
                ElevatorMotor.set_D(0);//0
                ElevatorMotor.set_allowedClosedLoopError(0.15);//0.15
                ElevatorMotor.set_maxVelocity(2000);//2500
                ElevatorMotor.set_maxAcceleration(2000*5);// 2500*5
        }

        //----------------------------------------------------------------
        // Methods
        //----------------------------------------------------------------
        public void Zero(){
                ElevatorMotor.SetZero();
        }

        public void Setposition(double height){
                ElevatorMotor.Set_position(height * 0.147058824);
        }
        public void Stop(){
                motor.stopMotor();
        }

}
