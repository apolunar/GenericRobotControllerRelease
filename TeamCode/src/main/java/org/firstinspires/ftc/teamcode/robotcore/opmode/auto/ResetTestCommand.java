package org.firstinspires.ftc.teamcode.robotcore.opmode.auto;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.command.lift.SetLift;
import org.firstinspires.ftc.teamcode.robotcore.command.chassis.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;

import java.util.ArrayList;

@Autonomous
@Disabled
public class ResetTestCommand extends CommandOpMode {
    @Override
    public void initialize() {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);

//        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
//                new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
//                new ArrayList<>(),
//                new Pose2d(-ObstacleMap.INCHES_PER_TILE * 2, 0, Rotation2d.fromDegrees(180)),
//                MyRobot.RobotConfig.trajectoryConfig
//        );
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, Rotation2d.fromDegrees(-90)),
                new ArrayList<>(),
                new Pose2d(0, -ObstacleMap.INCHES_PER_TILE * 2, Rotation2d.fromDegrees(-90)),
                AutoConfig.TRAJECTORY_CONFIG
        );

        schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SetLift(liftSubsystem, LiftSubsystem.DELIVERY_POSITION, 2000),
                                new FollowTrajectory(
                                        chassisSubsystem,
                                        AutoConfig.controller,
                                        new Pair<>(trajectory, Rotation2d.fromDegrees(0))

                                )
                        )
//                        new ResetLift(liftSubsystem)
//                        new FollowTrajectory(
//                                chassisSubsystem,
//                                MyRobot.RobotConfig.controller,
//                        )
                )
        );
    }
}
