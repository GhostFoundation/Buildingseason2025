package frc.robot.Library;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class TalonFX_SpeedCtrl{
    //----------------------------------------------------------------
    // Attributes
    //----------------------------------------------------------------
    public cs CS = new cs();
    public sts STS = new sts();
    public alm ALM = new alm();
    public par PAR = new par();
    private boolean Button_Pressed = false; //Variable which checks the button has been released before
    
    // Control Signal
    public class cs{ 
        public double SetSpeed = 0; // Motor Speed Setpoint in percentage (Speed changed in Shooter subsystem.)
    }

    // Status
    public class sts{
        private boolean Enabled = true;
        private boolean Init = false;
        private boolean Started = false;; // Motor has started
        private boolean Stopped = false;; // Motor has stopped
        private double ActualSpeed = 0; // The current speed of the motor in RPM

        public boolean Enabled(){
            return Enabled;
        }

        public boolean Init(){
            return Init;
        }

        public boolean Started(){
            return Started;
        }

        public boolean Stopped(){
            return Stopped;
        }

        public double ActualSpeed(){
            return ActualSpeed;
        }

    }

    // Parameter
    public class par{
        public TalonFX Motor; //Motor name for later declaration
        public boolean Enabled;
        public int Can_id; // The Can-id from the motor
        public boolean SetInverted = false;
    }

    // Alarm
    public class alm{

    }

    //----------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------
    /**
     * Constructor for the TalonFX Speed Controller
     * @param Can_id The Can-id of the motor
     * @param Enabled Enabled status when constructed, true for enabled when constructed
     * @param Inverted Invert mode of the motor. True for inverted (Positive speed for counterclockwise)
     */
    public TalonFX_SpeedCtrl(int Can_id, boolean Enabled, boolean Inverted){
        PAR.Enabled = Enabled;
        PAR.Can_id = Can_id;
        PAR.SetInverted = Inverted;
        PAR.Motor = new TalonFX(PAR.Can_id);

        if(PAR.Enabled == false){
            disabled();
        } else {
            Enabled();
        }
    }

    //----------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------
    private void GetRPM(){
        var speed = PAR.Motor.getVelocity();
        STS.ActualSpeed = speed.getValueAsDouble() * 60; // [RPM]

        if(STS.ActualSpeed > 0){
            STS.Stopped = false;
            STS.Started = true;
        }
        else{
            STS.Started = false;
            STS.Stopped = true;
        }
    }

    private void Init(){
        if(STS.Enabled && !STS.Init){
            TalonFXConfiguration config = new TalonFXConfiguration();
            if (PAR.SetInverted) {
                config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            } else {
                config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            }
            //config.MotorOutput.Inverted = PAR.SetInverted? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;  Short hand if statement
            
            //Motor.setInverted(PAR.SetInverted); //Will work until 2026
            PAR.Motor.getConfigurator().apply(config); //Apply the configuration to the motor
            STS.Init = true;
        }
    }

    private void Periodic(){
        if(STS.Enabled && STS.Init){
            GetRPM();
        }
    }

    public void Enabled(){
        STS.Enabled = true;
        Init();
        Periodic();
    }

    public void disabled(){
        PAR.Motor.disable();
        STS.Enabled = false;
    }
    
    /**
     * The motor is constantly turning when method is used.
     */
    public void Start(){
        if(STS.Enabled){
            PAR.Motor.set(CS.SetSpeed);
            Periodic();
        } 
    }
    
    /**
     * The motor stops when method is used.
     */
    public void Stop(){
        PAR.Motor.stopMotor();
    }

    /**
     * When "controllertrigger" is true the the motor will start. If false the motor will stop.
     * @param ControllerTrigger The switch for the trigger
     */ 
    public void StateActivation(boolean ControllerTrigger){
        if(STS.Enabled){
            if(ControllerTrigger){
                Start();
            }
            else{
                Stop();
            }
        }
    }

    /**
     * Whem “controllertrigger” is pressed and released the motor will set ON, when pressing again then it will stop.
     * @param ControllerTrigger The button which will activate trigger
     */
    public void Trigger(boolean ControllerTrigger){
        if(STS.Enabled){
            if (ControllerTrigger && !Button_Pressed) { //check if the button has been held, and button have not been pressed before
                if (PAR.Motor.get() == 0){ //check if the motor is set to stop, and set the oposite to happen
                    Start();
                    Button_Pressed = true;
                } else {
                    Stop();
                    Button_Pressed = true;
                }
            } else if (!ControllerTrigger){ //if no button is pressed, and the state of the button is still true, reset to false
                Button_Pressed = false;
            }
        }
    }
}
