package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import frc.robot.Library.*;
import frc.robot.Constants;

public class ArmSubsystem {
    //----------------------------------------------------------------
    // Attributes
    //----------------------------------------------------------------
    SparkMax Rotational_Motor = new SparkMax(Constants.ArmConstants.rotationMotorCanId, SparkMax.MotorType.kBrushless);
    //----------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------
    public ArmSubsystem(){
        
    }
}
