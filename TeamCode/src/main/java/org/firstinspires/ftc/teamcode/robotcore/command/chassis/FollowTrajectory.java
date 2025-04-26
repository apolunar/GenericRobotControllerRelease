package org.firstinspires.ftc.teamcode.robotcore.command.chassis;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.controller.DriveController;
import org.firstinspires.ftc.teamcode.math.trajectory.TrajectorySegment;
import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

public class FollowTrajectory extends CommandBase {
    private final ChassisSubsystem chassisSubsystem;
    private final DriveController controller;

    private final Pair<Trajectory, Rotation2d> trajectoryUnit;
    // TODO: Add as constructor default to keep things DRY
    private final Pose2d poseTolerance;
    private final double runtimeTolerancePercent;
    private final boolean disableCollisionDetection;

    private ElapsedTime runtime;
    private boolean firstRun = true;

    // TODO: Is there any way to describe this to lombok (leaving out the auto-generated constructor including the tolerance)
    public FollowTrajectory(ChassisSubsystem chassisSubsystem, DriveController controller, Pair<Trajectory, Rotation2d> trajectoryUnit, Pose2d poseTolerance, double runtimeTolerancePercent, boolean disableCollisionDetection) {
        this.chassisSubsystem = chassisSubsystem;
        this.controller = controller;
        this.trajectoryUnit = trajectoryUnit;
        // TODO: In these "utility commands" we keep defining things in terms of MyRobot
        // if this should be transferred to a public library users should need to initialize a RobotConfig w/constants and define it for the library to use
        // this way you don't have to keep saying/passing the same thing over and over again
        this.poseTolerance = poseTolerance;
        this.runtimeTolerancePercent = runtimeTolerancePercent;
        this.disableCollisionDetection = disableCollisionDetection;
    }

    public FollowTrajectory(ChassisSubsystem chassisSubsystem, DriveController driveController, Pair<Trajectory, Rotation2d> trajectoryUnit, Pose2d poseTolerance, double runtimeTolerancePercent) {
        this(chassisSubsystem, driveController, trajectoryUnit, poseTolerance, runtimeTolerancePercent, false);
    }

    public FollowTrajectory(ChassisSubsystem chassisSubsystem, DriveController driveController, Pair<Trajectory, Rotation2d> trajectoryUnit, Pose2d poseTolerance) {
        this(chassisSubsystem, driveController, trajectoryUnit, poseTolerance, AutoConfig.DEFAULT_TRAJECTORY_RUNTIME_TOLERANCE, false);
    }

    public FollowTrajectory(ChassisSubsystem chassisSubsystem, DriveController driveController, Pair<Trajectory, Rotation2d> trajectoryUnit, double runtimeTolerancePercent) {
        this(chassisSubsystem, driveController, trajectoryUnit, AutoConfig.DEFAULT_POSE_TOLERANCE, runtimeTolerancePercent, false);
    }

    public FollowTrajectory(ChassisSubsystem chassisSubsystem, DriveController driveController, Pair<Trajectory, Rotation2d> trajectoryUnit) {
        this(chassisSubsystem, driveController, trajectoryUnit, AutoConfig.DEFAULT_TRAJECTORY_RUNTIME_TOLERANCE);
    }

    @Override
    public void initialize() {
        RobotLog.dd(this.m_name, "========== Starting FollowTrajectory ==========");
//        TrajectorySequence.logTrajectory(trajectoryUnit.first);
    }

    @Override
    public void execute() {
        if (firstRun) {
            runtime = new ElapsedTime();
            firstRun = false;
        }

        Trajectory.State target = trajectoryUnit.first.sample(runtime.seconds());
//        RobotLog.dd(this.getClass().getSimpleName(), "Target : %s Runtime : %s", target, runtime.seconds());

        ChassisSpeeds speeds = controller.calculate(
                chassisSubsystem.getLatestPose(),
                target,
                trajectoryUnit.second
        );
//        RobotLog.dd(this.getClass().getSimpleName(), "ChassisSpeeds : (vx: %s vy: %s vt: %s)", speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);

        chassisSubsystem.driveChassisSpeeds(speeds);
    }

    @Override
    public void end(boolean good) {
        chassisSubsystem.driveChassisSpeeds(
                new ChassisSpeeds(0,0,0)
        );

        double estimatedTrajectoryTime = trajectoryUnit.first.getTotalTimeSeconds();
        RobotLog.dd(this.m_name, "Estimated trajectory time    : %s", estimatedTrajectoryTime);
        RobotLog.dd(this.m_name, "Actual trajectory time       : %s", runtime.seconds());
        RobotLog.dd(this.m_name, "Trajectory time pct accuracy : %s", (Math.abs(estimatedTrajectoryTime-runtime.seconds())/runtime.seconds())*100);
    }

    @Override
    public boolean isFinished() {
        boolean collision = false;
        if (!disableCollisionDetection) {
            // TODO: Check HolonomicDriveController for velocity calculations
//            MecanumDriveWheelSpeeds mecanumWheelSpeeds = chassisSubsystem.getMecanumWheelSpeeds();
//            OdoWheelSpeeds odoWheelSpeeds = chassisSubsystem.getOdoWheelSpeeds();
//
//            ChassisSpeeds mecanumChassisSpeeds = chassisSubsystem.getDriveKinematics().toChassisSpeeds(mecanumWheelSpeeds);
//
//            ChassisSpeeds odoChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
//                    odoWheelSpeeds.centerMetersPerSecond,
//                    (odoWheelSpeeds.leftMetersPerSecond + odoWheelSpeeds.rightMetersPerSecond) / 2,
//                    odoWheelSpeeds.leftMetersPerSecond,  - odoWheelSpeeds.rightMetersPerSecond);
        }

        boolean withinTolerance = controller.withinTolerance(chassisSubsystem.getLatestPose(), TrajectorySegment.getEndState(trajectoryUnit.first), trajectoryUnit.second, poseTolerance);

        double estimatedTrajectoryTime = trajectoryUnit.first.getTotalTimeSeconds();
        return collision || withinTolerance
                || (runtime.seconds() > estimatedTrajectoryTime+estimatedTrajectoryTime* runtimeTolerancePercent); // within tolerance, overtime, or collision
    }
}
