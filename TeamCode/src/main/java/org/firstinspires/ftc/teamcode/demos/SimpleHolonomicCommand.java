package org.firstinspires.ftc.teamcode.demos;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.controller.HolonomicDriveController;
import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

import java.util.ArrayList;

@Autonomous(group = "!Concept")
@Disabled
public class SimpleHolonomicCommand extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);

        Pose2d start = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d end = new Pose2d(24, -12, Rotation2d.fromDegrees(20));

        ArrayList<Translation2d> waypoints = new ArrayList<>();
//        waypoints.add(new Translation2d(12, 6));

        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(10, 5);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                start,
                waypoints,
                end,
                trajectoryConfig
        );

        final double kP = 1.0;
        HolonomicDriveController controller = new HolonomicDriveController(
//                new Pose2d(0.5, 0.5, new Rotation2d(0.1)),
                new PIDController(kP, 0, 0),
                new PIDController(kP, 0, 0),
                new ProfiledPIDController(kP, 0, 0,
                        new TrapezoidProfile.Constraints(10, 10))
//                new SimpleMotorFeedforward(1, 0., 0.),
//                new SimpleMotorFeedforward(1, 0., 0.)
        );

        // ---
        for (Trajectory.State state : trajectory.getStates()) {
            RobotLog.dd("TRAJECTORY", "State : %s", state);
        }
        // ---

        waitForStart();
        ElapsedTime runTimer = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            Trajectory.State target = trajectory.sample(runTimer.seconds());

            RobotLog.dd(this.getClass().getSimpleName(), "Target : %s", target);

            if (!controller.withinTolerance(chassisSubsystem.getLatestPose(), end, Rotation2d.fromDegrees(0), AutoConfig.DEFAULT_POSE_TOLERANCE)) {
                ChassisSpeeds speeds = controller.calculate(chassisSubsystem.getLatestPose(), target, target.poseMeters.getRotation());

                RobotLog.dd(this.getClass().getSimpleName(), "ChassisSpeeds : (vx: %s vy: %s vt: %s)", speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);

                chassisSubsystem.driveChassisSpeeds(speeds);
            } else {
                chassisSubsystem.driveChassisSpeeds(
                        new ChassisSpeeds(0,0,0)
                );
            }

        }
    }
}
