package frc.robot.subsystems;

//import java.util.List;

//import org.photonvision.PhotonCamera;
//import org.photonvision.targeting.PhotonTrackedTarget;
//import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.counter.EdgeConfiguration;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import frc.robot.Constants.Controller;
//import frc.robot.Library.FakePS4Controller;

public class Leds extends SubsystemBase {
    // Led lichtjes
    public AddressableLED led = new AddressableLED(0);
    public AddressableLEDBuffer m_ledbuffer = new AddressableLEDBuffer(12);

    // timer voor led
    public final Timer LedDelay = new Timer();

    public double counter;
    public boolean startTeller;

    public void VisionWithLED_Init() {
        // Voor led
        led.setLength(m_ledbuffer.getLength());
        // Set the data
        led.setData(m_ledbuffer);
        led.start();
        // timer voor led
        LedDelay.start();
    }

    public void Led_Strip_Solid(int R, int G, int B) {
        // dit is wanneer je intake aanzet dan gaat er geel lampje aan
        for (var i = 0; i < m_ledbuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for Geel
            m_ledbuffer.setRGB(i, R, G, B);
        }
        led.setData(m_ledbuffer);
    }

    public void Led_Strip_Knipper(int R, int G, int B, double KnipperInterval, double AantalKeerKnipper) {
        // dit is wanneer je de ir node herkent dan gaat voor 3 sec blauw knipperen
        if (LedDelay.get() < KnipperInterval) {
            for (var j = 0; j < m_ledbuffer.getLength(); j++) {
                // Sets the specified LED to the RGB values for blue
                m_ledbuffer.setRGB(j, R, G, B);
            }
            led.setData(m_ledbuffer);
        }
        if (LedDelay.get() > KnipperInterval) {
            for (var j = 0; j < m_ledbuffer.getLength(); j++) {
                // Sets the specified LED to the RGB values for red
                m_ledbuffer.setRGB(j, 0, 0, 0);
            }
            led.setData(m_ledbuffer);
            // LedDelay.reset();
        }

        if (LedDelay.get() > (KnipperInterval * 2)) {
            LedDelay.reset();
            if (counter <= AantalKeerKnipper) {
                startTeller = true;
                if (startTeller == EdgeConfiguration.kRisingEdge.rising) {
                    counter = counter + 1;
                }
            } else if (counter >= AantalKeerKnipper + 1.0) {
                startTeller = false;
                counter = AantalKeerKnipper + 1.0;
            }
        }

    }
}
