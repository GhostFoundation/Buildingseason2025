package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import frc.robot.Library.*;
import frc.robot.Constants;

public class ArmSubsystem {
    //----------------------------------------------------------------
    // Attributes
    //----------------------------------------------------------------
    SparkMax Rotational_Motor = new SparkMax(Constants.ArmConstants.kRotationMotorCanId, SparkMax.MotorType.kBrushless);
    NEO_Absolute_PID Rotational_PID = new NEO_Absolute_PID(Rotational_Motor);

    //----------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------
    public ArmSubsystem(){
        Rotational_PID.set_P(Constants.ArmConstants.kP);
        Rotational_PID.set_I(Constants.ArmConstants.kI);
        Rotational_PID.set_D(Constants.ArmConstants.kD);
        Rotational_PID.set_FF(Constants.ArmConstants.kFF);
        Rotational_PID.set_OutPutRange(Constants.ArmConstants.kOutputMin,Constants.ArmConstants.kOutputMax);
    }

}
