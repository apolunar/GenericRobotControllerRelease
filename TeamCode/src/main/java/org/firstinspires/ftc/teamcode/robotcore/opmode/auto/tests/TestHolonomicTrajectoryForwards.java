package org.firstinspires.ftc.teamcode.robotcore.opmode.auto.tests;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.command.chassis.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

import java.util.ArrayList;

@Autonomous
public class TestHolonomicTrajectoryForwards extends CommandOpMode {
    @Override
    public void initialize() {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);

        Pair<Trajectory, Rotation2d> seg = new Pair<>(
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                    new ArrayList<>(),
                    new Pose2d(2*ObstacleMap.INCHES_PER_TILE, 0, Rotation2d.fromDegrees(0)),
                    AutoConfig.TRAJECTORY_CONFIG
                ),
                Rotation2d.fromDegrees(0)
        );

        TrajectorySequence.logTrajectory(seg.first);

        schedule(
                new FollowTrajectory(
                        chassisSubsystem,
                        AutoConfig.controller,
                        seg,
                        1
                )
        );
    }
}
