package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CoralSubsystemConstants.ArmSetpoints;

public class CoralSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum Setpoint {
    kRest,
    kFeederStation,
    kLevel1,
    kLevel2,
    kLevel3,
    kLevel4;
  }

  // Initialize arm SPARK. We will use MAXMotion position control for the arm, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkMax armMotor =
      new SparkMax(25, MotorType.kBrushless);
  private SparkClosedLoopController armController = armMotor.getClosedLoopController();
  private RelativeEncoder armEncoder = armMotor.getAlternateEncoder();

  // Member variables for subsystem state management
  private double armCurrentTarget;
  


  public CoralSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
   

    // Zero arm encoders on initialization
    armEncoder.setPosition(0);
    

  }

  /**
   * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  private void moveToSetpoint() {
    armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
  }


  /**
   * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(Setpoint setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kRest:
                armCurrentTarget = ArmSetpoints.kRest;
                break;
            case kFeederStation:
              armCurrentTarget = ArmSetpoints.kFeederStation;
              break;
            case kLevel1:
              armCurrentTarget = ArmSetpoints.kLevel1;
              break;
            case kLevel2:
              armCurrentTarget = ArmSetpoints.kLevel2; 
              break;
            case kLevel3:
              armCurrentTarget = ArmSetpoints.kLevel3;
              break;
            case kLevel4:
              armCurrentTarget = ArmSetpoints.kLevel4;
              break;
          }
        });
  }




  @Override
  public void periodic() {
    moveToSetpoint();
    
  }

  
}