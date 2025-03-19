package frc.robot.Commands;

import frc.robot.subsystems.DriveSubsystem;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class VisionCommand extends Command {
@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem Swerve;
    public double tx;
    public double ty;
    public double ta;
    public double tv;
    public double tl;
    public boolean aligned;
  private final Timer timer = new Timer();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionCommand(DriveSubsystem subsystem) {
    this.Swerve = subsystem;
 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
    aligned = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0); 
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0.0);
    // if(tv==1){
    //     Command pathfindingCommand = AutoBuilder.pathfindToPose(new Pose2d(5,0,Rotation2d.fromDegrees(0)),
    //     new PathConstraints(3.0,4.0,Units.degreesToRadians(540), Units.degreesToRadians(720)),
    //     0.0);
        
    // }

    if(tv == 1 && aligned == false){
        
        if( tx-10 > -0.5 && tx-10 < 0.5){
            aligned = true;
        }else{
            Swerve.driveToTarget(0.075 * (tx-10), 0);
        }
    }else if(aligned == true){
        Swerve.driveToTarget(-0.75, 90);
    }
    
    }
    
    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

