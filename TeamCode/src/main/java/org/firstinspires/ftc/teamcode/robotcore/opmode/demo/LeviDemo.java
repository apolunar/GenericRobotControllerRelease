package org.firstinspires.ftc.teamcode.robotcore.opmode.demo;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.controller.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySegment;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.command.chassis.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

@TeleOp
@Disabled
public class LeviDemo extends CommandOpMode {
    private final double runtimeTolerance = 0.6;

    private TrajectoryConfig trajectoryConfig;
    private HolonomicDriveController controller;

    @Override
    public void initialize() {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);

        controller = AutoConfig.controller;

        double velAccel = 35;
        trajectoryConfig = new TrajectoryConfig(velAccel, velAccel);

        trajectoryConfig.setStartVelocity(0);
        trajectoryConfig.setEndVelocity(0);

        TrajectorySequence trajectorySequence = TrajectorySequence.weaveTrajectorySequence(
                new TrajectorySegment[]{
                        new TrajectorySegment(
                                Rotation2d.fromDegrees(-90),
                                new Translation2d[0],
                                new Pose2d(ObstacleMap.INCHES_PER_TILE*2.2, ObstacleMap.INCHES_PER_TILE*-3.2, Rotation2d.fromDegrees(-90)),
                                Rotation2d.fromDegrees(-90),
                                trajectoryConfig
                        ),
                        new TrajectorySegment(
                                Rotation2d.fromDegrees(180),
                                new Translation2d[0],
                                new Pose2d(
                                        ObstacleMap.INCHES_PER_TILE*0.8,
                                        ObstacleMap.INCHES_PER_TILE*-3.2, Rotation2d.fromDegrees(180)
                                ),
                                Rotation2d.fromDegrees(-90),
                                trajectoryConfig
                        ),
                        new TrajectorySegment(
                                Rotation2d.fromDegrees(0),
                                new Translation2d[0],
                                new Pose2d(ObstacleMap.INCHES_PER_TILE*2.0, ObstacleMap.INCHES_PER_TILE*-3.5, Rotation2d.fromDegrees(0)),
                                Rotation2d.fromDegrees(-90),
                                trajectoryConfig
                        )
                }
        );

        TrajectorySequence.logTrajectorySequence(trajectorySequence);


        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new FollowTrajectory(
                    chassisSubsystem,
                    controller,
                    trajectorySequence.get(0),
                    runtimeTolerance
                )
            )
        );

        register(
            chassisSubsystem
        );
    }
}
