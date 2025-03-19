package frc.robot.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Limelight{

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public void updateLimelightData() {
        NetworkTableEntry tv = table.getEntry("tv");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tl = table.getEntry("tl");
        NetworkTableEntry ts = table.getEntry("ts");

        double v = tv.getDouble(0.0); // 1 if target is present, 0 if not
        double x = tx.getDouble(0.0); // Horizontal offset from crosshair to target (-27 to 27 degrees)
        double y = ty.getDouble(0.0); // Vertical offset from crosshair to target (-20.5 to 20.5 degrees)
        double area = ta.getDouble(0.0); // Target area (0% to 100% of the image)
        double latency = tl.getDouble(0.0); //The pipeline’s latency contribution (ms)
        double skew = ts.getDouble(0.0); //Skew or rotation (-90 degrees to 0 degrees)

        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("LimelightLatency", latency);
        SmartDashboard.putNumber("Limelight Valid Target", v);
        // SmartDashboard.putNumber("Limelight Skew", ts);
    }
}

