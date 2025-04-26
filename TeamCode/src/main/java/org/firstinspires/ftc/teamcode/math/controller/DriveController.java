package org.firstinspires.ftc.teamcode.math.controller;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;

public interface DriveController {
    boolean withinTolerance(Pose2d currentPose, Pose2d targetPose, Rotation2d desiredHeading, Pose2d poseTolerance);

    ChassisSpeeds calculate(
            Pose2d currentPose,
            Pose2d trajectoryPose,
            double desiredLinearVelocityMetersPerSecond,
            Rotation2d desiredHeading);

    ChassisSpeeds calculate(
            Pose2d currentPose, Trajectory.State desiredState, Rotation2d desiredHeading);

    ProfiledPIDController getVtController();

    PIDController getVxController();

    PIDController getVyController();
}
