package frc.robot.Library;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;


public class NEO_Relative_PID {
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
            position = PAR.Encoder.getPosition()*360;
        }

        /**
         * Return the current position of the encoder
         * @return Current position of the encoder in degrees
         */
        public double get_position(){
            update_position();
            return position;
        }
    }

    //paramaters
    public class par{
        private SparkMax motor;
        private SparkClosedLoopController PIDController;
        private RelativeEncoder Encoder;
        
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
        public RelativeEncoder getEncoder(){
            return Encoder;
        }
    }

    //----------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------
    public NEO_Relative_PID(SparkMax motor){
        PAR.motor = motor;
        PAR.PIDController = PAR.motor.getClosedLoopController();
        PAR.Encoder = PAR.motor.getAlternateEncoder();
    }

    //----------------------------------------------------------------
    // Methods
    //----------------------------------------------------------------
    /**
     * Apply the PID configurations to the motor
     */
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
     * Store the current position of the encoder in STS
     */
    private void update_position(){
        STS.position = PAR.Encoder.getPosition()*360;
    }

    /**
     * Set the Porportional gain of the PID controller
     * @param P_gain Porportional gain value
     */
    public void set_P(double P_gain){
        STS.P_Gain = P_gain;
        setconfig();
    }

    /**
     * Set the Integral gain of the PID controller
     * @param I_gain Intergral gain value
     */
    public void set_I(double I_gain){
        STS.I_Gain = I_gain;
        setconfig();
    }

    /**
     * Set the Derivative gain of the PID controller 
     * @param D_gain Derivative gain value
     */
    public void set_D(double D_gain){
        STS.D_Gain = D_gain;
        setconfig();
    }

    /**
     * Set the Feed Forward gain of the PID controller
     * @param FF_gain Feed Forward gain value
     */
    public void set_FF(double FF_gain){
        STS.FF_Gain = FF_gain;
        setconfig();
    }

    /**
     * Set the output range of the PID controller
     * @param min Minimum output value (min -1)
     * @param max Maximum output value (max 1)
     */
    public void set_OutPutRange(double min, double max){
        STS.Output_Min = min;
        STS.Output_Max = max;
        setconfig();
    }

    /**
     * Set the current position of the encoder to 0
     */
    public void set_zero_position(){
        PAR.Encoder.setPosition(0);
    }

    /**
     * Go to the position of the encoder and hold it there. Updates the position variable in the mean while
     * @param pos The position on the encoder in degrees
     */
    public void set_pos_position(double pos) {
        pos = pos/360; //convert degrees to rotations
        PAR.PIDController.setReference(pos, ControlType.kPosition);
        update_position();
    }
}
