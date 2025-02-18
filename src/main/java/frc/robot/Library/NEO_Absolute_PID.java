package frc.robot.Library;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;


public class NEO_Absolute_PID {
    public cs CS = new cs();
    public sts STS = new sts();
    public par PAR = new par();

    //Control Signal
    public class cs{}
    
    //Status
    public class sts{
        private double P_Gain;
        private double I_Gain;
        private double D_Gain;
        private double FF_Gain;
        private double Output_Min;
        private double Output_Max;
        private double position;
        private double maxVelocity;
        private double maxAcceleration;
        private double allowedClosedLoopError;
        
        
        /**
         * Get the Porportional gain of the PID controller
         * @return Current Porportional gain value
         */
        public double get_P(){
            return P_Gain;
        }

        /**
         * Get the Integral gain of the PID controller
         * @return Current Integral gain value
         */
        public double get_I(){
            return I_Gain;
        }

        /**
         * Get the Derivative gain of the PID controller
         * @return Current Derivative gain value
         */
        public double get_D(){
            return D_Gain;
        }

        /**
         * Get the Feed Forward gain of the PID controller
         * @return Current Feed Forward gain value
         */
        public double get_FF(){
            return FF_Gain;
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

        /**
        * Store the current position of the encoder in STS
        */
        private void update_position(){
            position = PAR.Encoder.getPosition();
        }

        /**
         * Return the current position of the encoder
         * @return Current position of the encoder in degrees
         */
        public double get_position(){
            update_position();
            return position;
        }

        /**
         * Return the max velocity value of the PID controller
         * @return Current position of the encoder in degrees
         */
        public double get_maxVelocity(){
            return maxVelocity;
        }

        /**
         * Return the max acceleration value of the PID controller
         * @return Current position of the encoder in degrees
         */
        public double get_maxAcceleration(){
            return maxAcceleration;
        }

        /**
         * Return the allowed closed loop error value of the PID controller
         * @return Current position of the encoder in degrees
         */
        public double get_allowedClosedLoopError(){
            return allowedClosedLoopError;
        }
    }

    //paramaters
    public class par{
        private SparkMax motor;
        private SparkClosedLoopController PIDController;
        private SparkAbsoluteEncoder Encoder;
        public double P_Gain;
        public double I_Gain;
        public double D_Gain;
        public double FF_Gain;
        public double Output_Min;
        public double Output_Max;
        public double position;
        public double maxVelocity;
        public double maxAcceleration;
        public double allowedClosedLoopError;
        
        /**
         * Return the PID Controller object of the motor
         * @return PID Controller object
         */
        public SparkClosedLoopController getPIDController(){
            return PIDController;
        }

        /**
         * Return the Absolute Encoder object of the motor
         * @return Absolute Encoder object
         */
        public SparkAbsoluteEncoder getEncoder(){
            return Encoder;
        }
    }

    //----------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------
    public NEO_Absolute_PID(SparkMax motor){
        PAR.motor = motor;
        PAR.PIDController = PAR.motor.getClosedLoopController();
        PAR.Encoder = PAR.motor.getAbsoluteEncoder();
        STS.P_Gain = PAR.P_Gain;
        STS.I_Gain = PAR.I_Gain;
        STS.D_Gain = PAR.D_Gain;
        STS.FF_Gain = PAR.FF_Gain;
        STS.Output_Min = PAR.Output_Min;
        STS.Output_Max = PAR.Output_Max;
        STS.maxVelocity = PAR.maxVelocity;
        STS.maxAcceleration = PAR.maxAcceleration;
        STS.allowedClosedLoopError = PAR.allowedClosedLoopError;
    }

    //----------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------
    private void setconfig(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
        .p(STS.P_Gain)
        .i(STS.I_Gain)
        .d(STS.D_Gain)
        .outputRange(STS.Output_Min, STS.Output_Max)
        .velocityFF(STS.FF_Gain);

        PAR.motor.configure(config, null, null);
    }

    /**
     * Go to the position of the encoder and hold it there
     * @param pos The position on the encoder
     */
    public void set_position(double pos) {
        PAR.PIDController.setReference(pos, ControlType.kPosition);
    }
}
