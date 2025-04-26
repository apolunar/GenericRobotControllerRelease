package org.firstinspires.ftc.teamcode.robotcore.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.math.controller.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LimelightSubsystem;

import java.util.List;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AlignSampleLimelight extends CommandBase {
    private final ChassisSubsystem chassisSubsystem;
    private final ExtensionSubsystem extensionSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final HolonomicDriveController controller;

    private static final double CHASSIS_ERROR = 1;
    private static final double EXTENSION_OFFSET = 200; // Ticks for roll

    private LLResultTypes.DetectorResult targetSample = null;
    private boolean firstRun = true;

    @Override
    public void execute() {
        if (firstRun) {
            firstRun = false;
            limelightSubsystem.start();
            limelightSubsystem.setCurrentPipeline(LimelightSubsystem.LimelightPipeline.DETECTOR);
        }

        List<LLResultTypes.DetectorResult> results = limelightSubsystem.getDetectorResults();
        if (!results.isEmpty()) {
            // Every loop for the feedback controller
            // Sort for best target
            limelightSubsystem.getDetectorResults().sort(((a, b) -> {
                // Distance from crosshair
                double aDist = Math.hypot(a.getTargetXPixels(), a.getTargetYPixels());
                double bDist = Math.hypot(b.getTargetXPixels(), b.getTargetYPixels());

                // Target area
                double aArea = a.getTargetArea();
                double bArea = b.getTargetArea();

                // Combined score
                // Higher area is good, smaller distance is good
                double aScore = aArea / (aDist + 1); // +1 to avoid division by 0
                double bScore = bArea / (bDist + 1);

                return Double.compare(bScore, aScore); // Descending order
            }));
            targetSample = results.get(0);

            double error = Math.hypot(targetSample.getTargetXPixels(), targetSample.getTargetYPixels());

            if (error < CHASSIS_ERROR) {
                // This is dumb and would be stupid if it worked
                chassisSubsystem.driveChassisSpeeds(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                -controller.getVxController().calculate(targetSample.getTargetYPixels(), 0),
                                -controller.getVxController().calculate(targetSample.getTargetYPixels(), 0),
                                0,
                                chassisSubsystem.getLatestPose().getRotation()
                        )
                );
            }
            else {
                chassisSubsystem.drive(0, 0, 0);

                extensionSubsystem.setIntakePosition(0, 0);
            }

        }
    }

    public static double computeSlideExtension(
            double xPixel, double yPixel,
            double fx, double fy, double cx, double cy,
            double cameraHeightMeters,
            double cameraPitchRadians
    ) {
        double xCam = (xPixel - cx) / fx;
        double yCam = (yPixel - cy) / fy;

        double[] rayCamera = {xCam, yCam, 1.0};

        double sinPitch = Math.sin(cameraPitchRadians);
        double cosPitch = Math.cos(cameraPitchRadians);

        double rayX = rayCamera[0];
        double rayY = cosPitch * rayCamera[1] - sinPitch * rayCamera[2];
        double rayZ = sinPitch * rayCamera[1] + cosPitch * rayCamera[2];

        double t = -cameraHeightMeters / rayY;

        double horizontalDistance = t * Math.sqrt(rayX * rayX + rayZ * rayZ);

        return horizontalDistance;
    }
}
