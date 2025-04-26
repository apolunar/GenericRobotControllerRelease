package org.firstinspires.ftc.teamcode.robotcore.command.action;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Consumer;
import org.firstinspires.ftc.teamcode.logging.LogWrapper;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class TrajectoryToAprilTag extends CommandBase {
    private final ChassisSubsystem chassisSubsystem;

    private final CameraSubsystem cameraSubsystem;
    private final CameraSubsystem.RobotCamera camera;

    // TODO: Add support for "nearest tag"
    private final int targetTagId;
    private final Pose2d targetTagOffset;

    private final Consumer<Trajectory> trajectoryConsumer;

    private final TrajectoryConfig trajectoryConfig;

    private Pose2d target;
    private Trajectory targetTrajectory;

    @Override
    public void execute() {
        List<AprilTagDetection> detections = cameraSubsystem.getCurrentDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId) {
                Pose2d latestPose = chassisSubsystem.getLatestPose();

                Translation2d tag = new Translation2d(
                        camera == CameraSubsystem.RobotCamera.LIFT_CAM ? -detection.ftcPose.x : detection.ftcPose.x,
                        detection.ftcPose.y
                );

                Translation2d rotatedTarget = tag.rotateBy(new Rotation2d(
                        Math.PI/2 - latestPose.getHeading() - camera.cameraOffset.getRotation().getRadians())
                );

                target = new Pose2d(
                        latestPose.getX() - rotatedTarget.getX() - targetTagOffset.getX(),
                        latestPose.getY() + rotatedTarget.getY() - targetTagOffset.getY(),
                        targetTagOffset.getRotation()
                );

                targetTrajectory = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(latestPose.getX(), latestPose.getY(), targetTagOffset.getRotation()),
                        new ArrayList<>(),
                        target,
                        trajectoryConfig
                );

                TrajectorySequence.logTrajectory(targetTrajectory);
            }
        }
    }

    @Override
    public void end(boolean good) {
        RobotLog.dd(this.m_name, "Found april tag %s", targetTagId);
        LogWrapper.log(new LogWrapper.LogTarget[]{LogWrapper.LogTarget.TELEMETRY, LogWrapper.LogTarget.LOGCAT}, this.m_name, "Found april tag: %s", targetTagId);

        // TODO: The user of this command now has to do this manually to decrease init times
//        cameraSubsystem.teardownVisionPortal();

        trajectoryConsumer.accept(targetTrajectory);
    }

    @Override
    public boolean isFinished() {
        return target != null;
    }
}
