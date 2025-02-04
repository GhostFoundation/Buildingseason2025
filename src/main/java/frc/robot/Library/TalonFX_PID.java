package frc.robot.Library;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;

public class TalonFX_PID {
    
    sts STS = new sts();
    cs CS = new cs();
    par PAR = new par();

    //Control Signal
    public class cs {
    }
    
    //Status
    public class sts{
        private double P_Gain;
        private double I_Gain;
        private double D_Gain;
        private double FF_Gain;
        private double Output_Min;
        private double Output_Max;
        
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
    }

    public class par {
        public TalonFX Motor;
    }

    //----------------------------------------------------------------
    // Constructor
    //----------------------------------------------------------------
    public TalonFX_PID(TalonFX_SpeedCtrl motor){
        PAR.Motor = motor.PAR.Motor;
    }

    public void setconfig(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        TalonFXConfigurator Configurator = PAR.Motor.getConfigurator(); //Change later Paramaters
        config.Slot0.kP = STS.P_Gain;
        config.Slot0.kI = STS.I_Gain;
        config.Slot0.kD = STS.D_Gain;
        Configurator.apply(config);
        //Still working on how to set FF and output range.
    }

    public void set_P(double P){
        STS.P_Gain = P;
    }

    public void set_I(double I){
        STS.I_Gain = I;
    }

    public void set_D(double D){
        STS.D_Gain = D;
    }

    public void set_pos(double pos){
        double rotations = pos/360;
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        PAR.Motor.setControl(m_request.withPosition(rotations));
    }


    
}