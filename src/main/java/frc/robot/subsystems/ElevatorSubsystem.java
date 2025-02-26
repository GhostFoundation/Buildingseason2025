package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Library.*;

public class ElevatorSubsystem{
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
