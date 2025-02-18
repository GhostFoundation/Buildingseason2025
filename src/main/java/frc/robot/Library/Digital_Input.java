package frc.robot.Library;

import edu.wpi.first.wpilibj.DigitalInput;

public class Digital_Input {
    //----------------------------------------------------------------
    // Attributes
    //----------------------------------------------------------------
    public sts STS = new sts();
    public par PAR = new par();

    //Status
    public class sts {
        private boolean Status = false; //Variable storing status of the sensor

        /**
        * Update the detection status of the sensor, and return the value of the status.
        * Use Digital_Input Get_State() instead for simplicity.
        * @return
        * The status of the Sensor (On default false when nothing detected)
        */
        public boolean State(){
            Update_status();
            return Status;
        }
    }


    //Paramaters
    public class par {
        private DigitalInput Input;
        private int Port;
        private boolean Inverted = false; //Stores if the sensor should be inverted
    }


    /**
     * Contructor. Takes port number and create a DigitalInput Object. No inversion return false when reading HIGH, and vice versa
     * @param Port The Digital IO port number
     * @param Inverted Defualt to off, set true for inverse
     */
    public Digital_Input(int Port, boolean Inverted) {
        PAR.Port = Port;
        PAR.Inverted = Inverted;
        PAR.Input = new DigitalInput(PAR.Port);
    }


    /**
     * Check the status of the pin, and update status variable acordingly
     */
    private void Update_status(){
        //check for inverted or not and updpate status
        if (!PAR.Inverted) {
            STS.Status = (PAR.Input.get()) ? false : true; //set to false when nothing detected (e.g. IR Sensor read true when nothing is detected)
        } else {
            STS.Status = (PAR.Input.get()) ? true : false; //set to true when nothing detected (short hand if statement)
        }
    }

    public boolean Get_State() {
        return STS.State();
    }

}
