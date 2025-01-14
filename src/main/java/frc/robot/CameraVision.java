package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CameraVision {
    private static final double CAMERA_DISTANCE = 0.3;
    private static final double CX = 80.0; // Center X of the camera
    private static final double CY = 60.0; // Center Y of the camera
    private static final double FX = 160.0; // Focal length X
    private static final double FY = 160.0; // Focal length Y
    private static final double TAG_SIZE = 0.165; // Size of the AprilTag in meters

    private final UsbCamera leftCamera;
    private final UsbCamera rightCamera;
    private final CvSink leftCvSink;
    private final CvSink rightCvSink;
    private final CvSource outputStream;
    private final AprilTagDetector detector;

    public CameraVision(int leftCameraIndex, int rightCameraIndex) {
        leftCamera = CameraServer.startAutomaticCapture(leftCameraIndex);
        leftCamera.setResolution(160, 120); // Further reduced resolution
        leftCamera.setFPS(15); // Reduced frame rate
        leftCamera.setPixelFormat(PixelFormat.kMJPEG); // Change pixel format

        rightCamera = CameraServer.startAutomaticCapture(rightCameraIndex);
        rightCamera.setResolution(160, 120); // Further reduced resolution
        rightCamera.setFPS(15); // Reduced frame rate
        rightCamera.setPixelFormat(PixelFormat.kMJPEG); // Change pixel format

        leftCvSink = CameraServer.getVideo(leftCamera);
        rightCvSink = CameraServer.getVideo(rightCamera);
        outputStream = CameraServer.putVideo("Combined Camera", 320, 120); // Adjusted combined resolution

        detector = new AprilTagDetector();
        detector.addFamily("tag36h11");
    }

    public void detectTagsAndBalls() {
        Mat leftFrame = new Mat();
        Mat rightFrame = new Mat();

        if (leftCvSink.grabFrame(leftFrame) == 0 || rightCvSink.grabFrame(rightFrame) == 0) {
            leftFrame.release();
            rightFrame.release();
            return;
        }

        // Create a new Mat to hold the combined image
        Mat combinedFrame = new Mat(leftFrame.rows(), leftFrame.cols() + rightFrame.cols(), leftFrame.type());

        // Copy the left frame to the left side of the combined frame
        leftFrame.copyTo(combinedFrame.colRange(0, leftFrame.cols()));

        // Copy the right frame to the right side of the combined frame
        rightFrame.copyTo(combinedFrame.colRange(leftFrame.cols(), leftFrame.cols() + rightFrame.cols()));

        // Process the combined frame as needed
        Mat leftGray = new Mat();
        Mat rightGray = new Mat();
        Imgproc.cvtColor(leftFrame, leftGray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.cvtColor(rightFrame, rightGray, Imgproc.COLOR_BGR2GRAY);

        AprilTagDetection[] leftDetectionsArray = detector.detect(leftGray);
        AprilTagDetection[] rightDetectionsArray = detector.detect(rightGray);
        List<AprilTagDetection> leftDetections = new ArrayList<>(List.of(leftDetectionsArray));
        List<AprilTagDetection> rightDetections = new ArrayList<>(List.of(rightDetectionsArray));

        int totalTags = leftDetections.size() + rightDetections.size();
        SmartDashboard.putNumber("Total AprilTags", totalTags);

        for (AprilTagDetection leftDetection : leftDetections) {
            for (AprilTagDetection rightDetection : rightDetections) {
                if (leftDetection.getId() == rightDetection.getId()) {
                    double[] leftPose = estimatePose(leftDetection);
                    double[] rightPose = estimatePose(rightDetection);
                    double[] robotPosition = calculateRobotPosition(leftPose, rightPose);

                    SmartDashboard.putNumber("AprilTag ID: " + leftDetection.getId(), leftDetection.getId());
                    SmartDashboard.putNumber("Left Camera Distance: " + leftDetection.getId(), leftPose[0]);
                    SmartDashboard.putNumber("Right Camera Distance: " + leftDetection.getId(), rightPose[0]);
                    SmartDashboard.putNumber("Robot Position X: " + leftDetection.getId(), robotPosition[0]);
                    SmartDashboard.putNumber("Robot Position Y: " + leftDetection.getId(), robotPosition[1]);
                }
            }
        }

        int leftBallCount = detectBall(leftFrame);
        int rightBallCount = detectBall(rightFrame);
        int totalBalls = leftBallCount + rightBallCount;
        SmartDashboard.putNumber("Total Balls", totalBalls);

        // Output the combined frame to the stream
        outputStream.putFrame(combinedFrame);

        // Release resources
        leftFrame.release();
        rightFrame.release();
        combinedFrame.release();
        leftGray.release();
        rightGray.release();
    }

    private double[] estimatePose(AprilTagDetection detection) {
        double[] corners = detection.getCorners();
        double centerX = (corners[0] + corners[2] + corners[4] + corners[6]) / 4.0;
        double centerY = (corners[1] + corners[3] + corners[5] + corners[7]) / 4.0;

        double dx = (centerX - CX) / FX;
        double dy = (centerY - CY) / FY;

        double distance = TAG_SIZE / Math.sqrt(dx * dx + dy * dy);
        double angle = Math.toDegrees(Math.atan2(dy, dx));

        return new double[]{distance * 100, angle};
    }

    private double[] calculateRobotPosition(double[] leftPose, double[] rightPose) {
        double leftDistance = leftPose[0];
        double rightDistance = rightPose[0];

        double x = (leftDistance + rightDistance) / 2;
        double y = (rightDistance - leftDistance) / CAMERA_DISTANCE * TAG_SIZE;

        return new double[]{x * 100, y * 100};
    }

    private int detectBall(Mat frame) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

        Scalar lowerCyan = new Scalar(80, 100, 100);
        Scalar upperCyan = new Scalar(100, 255, 255);

        Mat mask = new Mat();
        Core.inRange(hsv, lowerCyan, upperCyan, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(frame, contours, -1, new Scalar(0, 255, 0), 2);

        int ballCount = contours.size();
        SmartDashboard.putNumber("Detected Balls", ballCount);

        // Release resources
        hsv.release();
        mask.release();
        kernel.release();

        return ballCount;
    }
}