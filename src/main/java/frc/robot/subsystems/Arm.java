package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Library.*;

public class Arm {
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
        private SparkMax motor = new SparkMax(25, MotorType.kBrushless);
        public NEO_Relative_PID Armmotor = new NEO_Relative_PID(motor);

        //----------------------------------------------------------------
        // Constructor
        //----------------------------------------------------------------
        public Arm(){
        
        }

        //----------------------------------------------------------------
        // Methods
        //----------------------------------------------------------------
        public void Zero(){
                Armmotor.SetZero();
        }

        public void Setposition(double pos_degrees){
                Armmotor.Set_position(pos_degrees);
        }

}