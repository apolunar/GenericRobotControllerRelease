package org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.logging.LogWrapper;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotConfig;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

import lombok.Getter;

@Getter
@Config
public class CameraSubsystem extends SubsystemBase implements RobotSubsystem {
    public static final String LIFT_CAM_NAME = "lift cam";
    public static final String DIFFY_CAM_NAME = "diffy cam";

    public enum RobotCamera {
        LIFT_CAM(new Pose2d(), VisionPortal.StreamFormat.MJPEG, 1920, 1200),
        DIFFY_CAM(new Pose2d(), VisionPortal.StreamFormat.MJPEG, 640,480);
        public Pose2d cameraOffset;
        public VisionPortal.StreamFormat streamFormat;
        public int width;
        public int height;
        public WebcamName webcamName;
        RobotCamera(Pose2d cameraOffset, VisionPortal.StreamFormat streamFormat, int width, int height) {
            this.cameraOffset = cameraOffset;
            this.streamFormat = streamFormat;
            this.width = width;
            this.height = height;
        }
    }

//    public final OpenCvCamera liftCam;
    public final OpenCvCamera diffyCam;

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTagProcessor;

    private List<AprilTagDetection> currentDetections;

    public static final Mat armCameraMatrix;
    public static final Mat armDistCoeffs;

    static {
        armCameraMatrix = new Mat(3, 3, CvType.CV_64F);
        // [ fx  0  cx ]
        // [  0  fy cy ]
        // [  0   0  1 ]
        armCameraMatrix.put(0, 0, 1242.16153862);  // fx
        armCameraMatrix.put(0, 1, 0);
        armCameraMatrix.put(0, 2, 955.027369152);  // cx
        armCameraMatrix.put(1, 0, 0);
        armCameraMatrix.put(1, 1, 1242.16153862);  // fy
        armCameraMatrix.put(1, 2, 471.037067075);  // cy
        armCameraMatrix.put(2, 0, 0);
        armCameraMatrix.put(2, 1, 0);
        armCameraMatrix.put(2, 2, 1);

        // The distortion coefficients are typically a 1x5 or 1x8 Mat
        armDistCoeffs = new Mat(1, 5, CvType.CV_64F);
        // Fill in the distortion coefficients as computed during calibration
        // [k1, k2, p1, p2, k3]
        armDistCoeffs.put(0, 0, -0.447930149901, 0.243617009491, 0.00301539085885, -0.00505585463232, -0.0717713075936);
    }

    public CameraSubsystem(WebcamName liftCam, WebcamName diffyCam, int cameraMonitorViewId) {
//        RobotCamera.LIFT_CAM.webcamName = liftCam;
//        this.liftCam = OpenCvCameraFactory.getInstance().createWebcam(liftCam);
        RobotCamera.DIFFY_CAM.webcamName = diffyCam;
        this.diffyCam = OpenCvCameraFactory.getInstance().createWebcam(diffyCam, cameraMonitorViewId);
    }

    public CameraSubsystem(HardwareMap hMap) {
        this(
//                hMap.get(WebcamName.class, LIFT_CAM_NAME),
                null,
                hMap.get(WebcamName.class, DIFFY_CAM_NAME),
                hMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName())
        );
    }

    public void setupVisionPortal(RobotCamera camera, VisionProcessor processor) {
//        // Create the AprilTag processor by using a builder
//        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        // Create the WEBCAM vision portal by using a builder
        visionPortal = new VisionPortal.Builder()
                .setCamera(camera.webcamName)
                .addProcessor(processor)
                .setCameraResolution(new Size(camera.width, camera.height))
                .enableLiveView(RobotConfig.DEBUG)
                .setStreamFormat(camera.streamFormat)
                .setAutoStopLiveView(true)
                .build(); // NOTE: Defaults to center but can be changed dynamically
    }

    public void setupAprilTagPortal(RobotCamera camera) {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();

        setupVisionPortal(camera, aprilTagProcessor);
    }

    public void shutdownVisionPortal() {
        visionPortal.close();
    }

    @Override
    public void periodic() {
        if (aprilTagProcessor == null) return;

        currentDetections = aprilTagProcessor.getDetections();
        LogWrapper.log(this.getName(), "# AprilTags Detected %s", currentDetections.size());

        // Step through the list of detections and display info for each one
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                LogWrapper.log(this.getName(), "\n==== (ID %d) %s", detection.id, detection.metadata.name);
                LogWrapper.log(this.getName(), "XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                LogWrapper.log(this.getName(), "PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);
                LogWrapper.log(this.getName(), "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);

                // TODO: Use detection.metadata.fieldPosition and detection.metadata.fieldOrientation to estimate robot position on field
            } else {
                LogWrapper.log(this.getName(), "\n==== (ID %d) Unknown", detection.id);
                LogWrapper.log(this.getName(), "Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y);
            }
        }
    }

    public void switchCamera(RobotCamera camera) {
        visionPortal.setActiveCamera(camera.webcamName);
    }

    public void openOpenCvCamera(OpenCvCamera camera, RobotCamera robotCamera) {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(robotCamera.width, robotCamera.height, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                RobotLog.ee(this.getClass().getSimpleName(), "ERROR! Could not open %s camera!", robotCamera.name());
            }
        });
    }
}
