package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class AprilTagVisionSubsystem extends SubsystemBase {
    private final PhotonCamera cameraLeft;
    private final PhotonCamera cameraRight;
    private final Transform3d cameraToRobot;
    private final AprilTagFieldLayout aprilTagFieldLayout;
                //names: leftCam rightCam, 
                //cameraToRobot: position and rotation of the cam relative to the center of the robot
                //for example: 
                //Transform3d cameraToRobot = new Transform3d(new Translation3d(0.1, 0.2, 0.3), new Rotation3d(0, 0, 0));
                //above is the translation xyz 10cm, 20cm, 30cm and rotation xyz 0, 0, 0
    public AprilTagVisionSubsystem(String leftCameraName, String rightCameraName, Transform3d cameraToRobot) {
        cameraLeft = new PhotonCamera(leftCameraName);
        cameraRight = new PhotonCamera(rightCameraName);
        this.cameraToRobot = cameraToRobot;
        this.aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    }

    public Pose3d calculateRobotPose() throws Exception {
        var leftResult = cameraLeft.getLatestResult();
        var rightResult = cameraRight.getLatestResult();

        if (!leftResult.hasTargets() && !rightResult.hasTargets()) {
            SmartDashboard.putString("apriltag error", "no apriltag detected by either camera");
            throw new Exception("no apriltag detected by either camera");
        }

        PhotonTrackedTarget leftTarget = leftResult.getBestTarget();
        PhotonTrackedTarget rightTarget = rightResult.getBestTarget();

        if (leftResult.hasTargets() && rightResult.hasTargets() && leftTarget.getFiducialId() == rightTarget.getFiducialId()) {
            int fiducialId = leftTarget.getFiducialId();
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(fiducialId);
            SmartDashboard.putString("tag pose: ", tagPose.toString() + " tag: " + fiducialId);            
            if (tagPose.isPresent()) {
                Pose3d robotPoseLeft = PhotonUtils.estimateFieldToRobotAprilTag(leftTarget.getBestCameraToTarget(), tagPose.get(), cameraToRobot);
                Pose3d robotPoseRight = PhotonUtils.estimateFieldToRobotAprilTag(rightTarget.getBestCameraToTarget(), tagPose.get(), cameraToRobot);
                //interpolates the cameras 0.5 meaning the robot pose is in the middle of the two cameras
                return robotPoseLeft.interpolate(robotPoseRight, 0.5);
            } else {
                SmartDashboard.putString("apriltag error", "apriltag pose not found in field layout.");
                throw new Exception("apriltag pose not found in field layout.");
            }
        } else if (leftResult.hasTargets()) {
            int fiducialId = leftTarget.getFiducialId();
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(fiducialId);
            SmartDashboard.putString("tag pose: ", tagPose.toString() + " tag: " + fiducialId);            

            if (tagPose.isPresent()) {
                return PhotonUtils.estimateFieldToRobotAprilTag(leftTarget.getBestCameraToTarget(), tagPose.get(), cameraToRobot);
            } else {
                SmartDashboard.putString("apriltag error", "apriltag pose not found in field layout.");
                throw new Exception("apriltag pose not found in field layout.");
            }
        } else {
            int fiducialId = rightTarget.getFiducialId();
            Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(fiducialId);
            SmartDashboard.putString("tag pose: ", tagPose.toString() + " tag: " + fiducialId);            
            if (tagPose.isPresent()) {
                return PhotonUtils.estimateFieldToRobotAprilTag(rightTarget.getBestCameraToTarget(), tagPose.get(), cameraToRobot);
            } else {
                SmartDashboard.putString("apriltag error", "apriltag pose not found in field layout.");
                throw new Exception("apriltag pose not found in field layout.");
            }
        }
    }

    @Override
    public void periodic() {
    }
}