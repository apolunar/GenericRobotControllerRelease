package org.firstinspires.ftc.teamcode.robotcore.opmode.auto.tests;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.math.controller.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.command.chassis.FollowTrajectory;
import org.firstinspires.ftc.teamcode.robotcore.game.enviroment.ObstacleMap;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

import java.util.ArrayList;

@Autonomous
@Config
public class TestHolonomicTrajectoryBackwards extends CommandOpMode {
    /**
     * Default PID values
     */
    public static double xkP = 7;
    public static double xkI = 0.03;
    public static double xkD = 0.04;

    public static double ykP = 7;
    public static double ykI = 0.03;
    public static double ykD = 0.05;

    public static double tkP = 3;
    public static double tkI = 1.0;
    public static double tkD = 0.001;

    public static int ANGULAR_VEL   = 600;
    public static int ANGULAR_ACCEL = 600;


    @Override
    public void initialize() {
        HolonomicDriveController controller = new HolonomicDriveController(
//                new Pose2d(1, 1, Rotation2d.fromDegrees(3)),
                new PIDController(xkP, xkI, xkD), // vx controller
                new PIDController(ykP, ykI, ykD), // vy controller
                new ProfiledPIDController(tkP, tkI, tkD, new TrapezoidProfile.Constraints(ANGULAR_VEL, ANGULAR_ACCEL)) // omega controller
        );

        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);

        Pair<Trajectory, Rotation2d> seg = new Pair<>(
                TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, Rotation2d.fromDegrees(-180)),
                    new ArrayList<>(),
                    new Pose2d(-2*ObstacleMap.INCHES_PER_TILE, 0, Rotation2d.fromDegrees(-180)),
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
