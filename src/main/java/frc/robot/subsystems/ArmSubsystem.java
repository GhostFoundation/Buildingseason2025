package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import frc.robot.Library.*;
import frc.robot.Constants;

public class ArmSubsystem {
    //----------------------------------------------------------------
    // Attributes
    //----------------------------------------------------------------
    SparkMax Rotational_Motor = new SparkMax(Constants.ArmConstants.kRotationMotorCanId, SparkMax.MotorType.kBrushless);
    NEO_Absolute_PID Motor = new NEO_Absolute_PID(Rotational_Motor);

    //----------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------
    public ArmSubsystem(){
        Motor.set_P(Constants.ArmConstants.kP);
        Motor.set_I(Constants.ArmConstants.kI);
        Motor.set_D(Constants.ArmConstants.kD);
        Motor.set_FF(Constants.ArmConstants.kFF);
        Motor.set_OutPutRange(Constants.ArmConstants.kOutputMin,Constants.ArmConstants.kOutputMax);
    }

    public void ArmPosition(FakePS4Controller controller){
        if(controller.getSquareButton()){ //change button mapping later!
            Motor.set_position(Constants.ArmConstants.kPosition1);
        }
        else if(controller.getTriangleButton()){
            Motor.set_position(Constants.ArmConstants.kPosition2);
        }
        
    }

}
