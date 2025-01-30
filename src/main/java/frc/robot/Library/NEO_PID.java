package frc.robot.Library;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;


public class NEO_PID {
    public cs CS = new cs();
    public sts STS = new sts();
    public par PAR = new par();

    //Control Signal
    public class cs{}
    
    //Status
    public class sts{
        private double kP;
        private double kI;
        private double kD;
        private double kFF;
        private double Output_Min;
        private double Output_Max;
        
        /**
         * Get the Porportional gain of the PID controller
         * @return Current Porportional gain value
         */
        public double get_P(){
            return kP;
        }

        /**
         * Get the Integral gain of the PID controller
         * @return Current Integral gain value
         */
        public double get_I(){
            return kI;
        }

        /**
         * Get the Derivative gain of the PID controller
         * @return Current Derivative gain value
         */
        public double get_D(){
            return kD;
        }

        /**
         * Get the Feed Forward gain of the PID controller
         * @return Current Feed Forward gain value
         */
        public double get_FF(){
            return kFF;
        }

        /**
         * Get the minimum output value of the PID controller
         * @return Current minimum output value
         */
        public double get_OutputMin(){
            return Output_Min;
        }

        /**
         * Get the maximum output value of the PID controller
         * @return Current maximum output value
         */
        public double get_OutputMax(){
            return Output_Max;
        }
    }

    //paramaters
    public class par{
        private SparkMax motor;
        private SparkClosedLoopController PIDController;
        private SparkAbsoluteEncoder Encoder;
        public double kP = 1;
        public double kI = 0;
        public double kD = 0;
        public double kFF = 0;
        public double minOutputRange = -1;
        public double maxOutputRange = 1;
        
        /**
         * Return the PID Controller object of the motor
         * @return PID Controller object
         */
        private SparkClosedLoopController getPIDController(){
            return PIDController;
        }

        /**
         * Return the Absolute Encoder object of the motor
         * @return Absolute Encoder object
         */
        private SparkAbsoluteEncoder getEncoder(){
            return Encoder;
        }
    }

    // Testing for elevator
    //private DCMotor elevatorMotorModel = DCMotor.getNEO(2);

    //----------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------
    public NEO_PID(NEO_SpeedCtrl motor){
        PAR.motor = motor.PAR.Motor;
        PAR.PIDController = PAR.motor.getClosedLoopController();
        PAR.Encoder = PAR.motor.getAbsoluteEncoder();

        
        STS.kP = PAR.kP;
        STS.kI = PAR.kI;
        STS.kD = PAR.kD;
        STS.kFF = PAR.kFF;
        STS.Output_Min = PAR.minOutputRange;
        STS.Output_Max = PAR.maxOutputRange;
    }

    //----------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------
    public void setconfig(){
        set_P(PAR.kP);
        set_I(PAR.kP);
        set_D(PAR.kP);
        set_FF(PAR.kP);
        set_OutPutRange(PAR.minOutputRange, PAR.maxOutputRange);

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
        .p(STS.kP)
        .i(STS.kI)
        .d(STS.kD)
        .outputRange(STS.Output_Min, STS.Output_Max)
        .velocityFF(STS.kFF);
        PAR.motor.configure(config, null, null);
    }

    /**
     * Set the Porportional gain of the PID controller
     * @param P_gain Porportional gain value
     */
    private void set_P(double P_gain){
        STS.kP = P_gain;
    }

    /**
     * Set the Integral gain of the PID controller
     * @param I_gain Intergral gain value
     */
    private void set_I(double I_gain){
        STS.kI = I_gain;
    }

    /**
     * Set the Derivative gain of the PID controller 
     * @param D_gain Derivative gain value
     */
    private void set_D(double D_gain){
        STS.kD = D_gain;
    }

    /**
     * Set the Feed Forward gain of the PID controller
     * @param FF_gain Feed Forward gain value
     */
    private void set_FF(double FF_gain){
        STS.kFF = FF_gain;
    }

    /**
     * Set the output range of the PID controller
     * @param min Minimum output value (min -1)
     * @param max Maximum output value (max 1)
     */
    private void set_OutPutRange(double min, double max){
        STS.Output_Min = min;
        STS.Output_Max = max;
    }

    /**
     * Go to the position of the encoder and hold it there
     * @param pos The position on the encoder
     */
    public void set_pos(double pos) {
        PAR.PIDController.setReference(pos, ControlType.kPosition);
    }
}
