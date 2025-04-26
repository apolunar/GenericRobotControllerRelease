package org.firstinspires.ftc.teamcode.robotcore.command.sequence;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;

import org.firstinspires.ftc.teamcode.math.controller.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.demos.MoveLift;
import org.firstinspires.ftc.teamcode.robotcore.command.action.TrajectoryToAprilTag;
import org.firstinspires.ftc.teamcode.robotcore.command.action.WaitSeconds;
import org.firstinspires.ftc.teamcode.robotcore.command.chassis.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robotcore.game.Alliance;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.demos.ExtendSubsystem;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class AlignAndDeposit extends SequentialCommandGroup {
    private final ChassisSubsystem chassisSubsystem;
    private final HolonomicDriveController controller;
    private final ExtendSubsystem extendSubsystem;
    private final CameraSubsystem cameraSubsystem;
    private final int targetBackboardTagId;
    private final Alliance alliance;
    private final double runtimeTolerance;

    @Override
    public void initialize() {
        addCommands(
                new TrajectoryToAprilTag(
                        chassisSubsystem, cameraSubsystem,
                        CameraSubsystem.RobotCamera.LIFT_CAM, targetBackboardTagId,
                        new Pose2d(-0.5, alliance == Alliance.RED_ALLIANCE ? 7.5 : -7.5, alliance == Alliance.RED_ALLIANCE ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(-90)),
                        (trajectory ->
                                CommandScheduler.getInstance().schedule(
                                        new SequentialCommandGroup(
                                                new FollowTrajectory(
                                                        chassisSubsystem, controller,
                                                        new Pair<>(trajectory, alliance == Alliance.RED_ALLIANCE ? Rotation2d.fromDegrees(90) : Rotation2d.fromDegrees(-90)),
                                                        runtimeTolerance
                                                ),
                                                new WaitSeconds(0.5),
                                                new MoveLift(extendSubsystem, 1600, 600)
                                        )
                                )
                        ),
                        new TrajectoryConfig(22, 30)
                )
        );
    }
}
