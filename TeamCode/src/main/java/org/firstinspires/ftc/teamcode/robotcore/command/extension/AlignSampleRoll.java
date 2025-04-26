package org.firstinspires.ftc.teamcode.robotcore.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.vision.AngleStabilizer;
import org.firstinspires.ftc.teamcode.robotcore.vision.SampleSampler;
import org.opencv.core.RotatedRect;

import lombok.Getter;
import lombok.Setter;

public class AlignSampleRoll extends CommandBase {
    private final ExtensionSubsystem extensionSubsystem;
    private final CameraSubsystem cameraSubsystem;
    private final Telemetry telemetry;

    private static final double RETARGET_TIME = 0.5;

    public SampleSampler sampleSampler;
    private final AngleStabilizer stabilizer = new AngleStabilizer(4); // Last X detections

    @Setter
    @Getter
    private boolean enableRoll = true;

    public AlignSampleRoll(ExtensionSubsystem extensionSubsystem, CameraSubsystem cameraSubsystem, Telemetry telemetry) {
        this.extensionSubsystem = extensionSubsystem;
        this.cameraSubsystem = cameraSubsystem;
        this.telemetry = telemetry;

        sampleSampler = new SampleSampler(telemetry);
        CameraSubsystem.RobotCamera camera = CameraSubsystem.RobotCamera.DIFFY_CAM;
        cameraSubsystem.setupVisionPortal(camera, sampleSampler);

//        // Create the WEBCAM vision portal by using a builder
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(camera.webcamName)
//                .setCameraResolution(new Size(camera.width, camera.height))
//                .enableLiveView(RobotConfig.DEBUG)
//                .setStreamFormat(camera.streamFormat)
//                .setAutoStopLiveView(true)
//                .addProcessor(sampleSampler)
//                .build();
    }

    @Getter
    private RotatedRect targetRect = null;
    private final ElapsedTime targetTimer = new ElapsedTime();
    private double angle = 0;

    @Override
    public void execute() {
        if (targetTimer.seconds() > RETARGET_TIME || targetRect == null) {
            RotatedRect newTargetRect = sampleSampler.getLargestRect();
            if (newTargetRect == null) {
                return;
            } // Maybe next time
            // Found!
            // TODO: Yaw?
            targetTimer.reset();
            targetRect = newTargetRect;
        } // Only retarget after a bit and if there's a rect to target

        // Roll
        double normalizedAngle = getNormalizedAngle(targetRect) * Math.PI / 180.0 - Math.PI/2;
        double newAngle = stabilizer.addAndGetStableAngle(normalizedAngle);
//        telemetry.addData("angle", Math.toDegrees(newAngle));

//        RobotLog.dd(this.m_name, "output: %s, dx: %s, dy: %s, distance: %s, angle: %s", output, dx, dy, distance, angle);
//        RobotLog.dd(this.m_name, "angle: %s", Math.toDegrees(newAngle));

        // Prevents the claw from tweaking out
        if (newAngle != angle) {
            angle = newAngle;
//            double finalClawRotation = (-Math.PI/2*1.2)/(Math.PI) + 0.5;
//            double clawRoll = ((finalClawRotation+0.1)%1.2)-0.1;
//            RobotLog.dd(this.m_name, "Set claw rotation to: %s", finalClawRotation);
//            RobotLog.dd(this.m_name, "Set claw roll to: %s", clawRoll);

            if (enableRoll) {
                extensionSubsystem.setClawAngle(-angle);
            }
        }

        // --- old stuff ---
        //        visionController = new PIDController(kP, kI, kD);

        // Yaw - TODO: Yaw is STILL too finicky to get the kind of fine control we need :(
//        double dx = targetRect.center.x - targetImagePoint.x;
//        double dy = targetRect.center.x - targetImagePoint.y;
//        double distance = Math.sqrt(dx * dx + dy * dy); // error, in this case
//
//        double output = visionController.calculate(distance, 0);
//        RobotLog.dd(this.m_name, "output: %s, dx: %s, dy: %s, distance: %s, angle: %s", output, dx, dy, distance, angle);
//
//        if (distance > 100) {
//            extensionSubsystem.driveYaw(output * Math.signum(dx));
//        } else {
//            extensionSubsystem.driveYaw(0);
//        }

//        extensionSubsystem.setClawRoll(0.6);
    }

    public static double getNormalizedAngle(RotatedRect rect) {
        double angle = rect.angle;
        org.opencv.core.Size size = rect.size;
        // Check if the width is less than the height.
        if (size.width < size.height) {
            // Adjust the angle by 90 degrees to account for the swap between width and height.
            angle += 90;
        }
        return angle;
    }
}
