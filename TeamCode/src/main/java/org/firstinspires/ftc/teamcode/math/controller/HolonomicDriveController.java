package org.firstinspires.ftc.teamcode.math.controller;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.qualcomm.robotcore.util.RobotLog;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * This follows trajectories using a holonomic drivetrain (swerve, mecanum, omni)
 * This is a simpler problem to solve than skid-steer (e.g. ramsete) because you can individually
 * control forward, sideways, and angular velocities.
 *
 * This takes a PID(F) controller for vx and vy because the system has some known characteristic
 * along with a profiled PID controller for angular because heading dynamics are decoupled from translations.
 * This is profiled for smoothness.
 */
@RequiredArgsConstructor
@Getter
public class HolonomicDriveController implements DriveController {
    private final PIDController vxController;
    private final PIDController vyController;
    private final ProfiledPIDController vtController;

    public boolean withinTolerance(Pose2d currentPose, Pose2d targetPose, Rotation2d desiredHeading, Pose2d poseTolerance) {
        Pose2d targetPoseError = targetPose.relativeTo(currentPose);
        Rotation2d targetRotationError = desiredHeading.minus(currentPose.getRotation());

//        RobotLog.vv(this.getClass().getSimpleName(), "Calculate errors     : (pose error : %s rotation error : %s)", targetPoseError, targetRotationError);

        return Math.abs(targetPoseError.getX()) < poseTolerance.getX()
                && Math.abs(targetPoseError.getY()) < poseTolerance.getY()
                && Math.abs(targetRotationError.getRadians()) < poseTolerance.getRotation().getRadians();
    }

    public ChassisSpeeds calculate(
            Pose2d currentPose,
            Pose2d trajectoryPose,
            double desiredLinearVelocityMetersPerSecond,
            Rotation2d desiredHeading) {
        // Calculate field-relative feedforward velocities by splitting velocity into components
        double vxTarget = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos();
        double vyTarget = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin();

        // Calculate feedback velocities
        double vxFeedback = -vxController.calculate(currentPose.getX(), trajectoryPose.getX());
        double vyFeedback = -vyController.calculate(currentPose.getY(), trajectoryPose.getY());

        double vtFeedback = -vtController.calculate(
                currentPose.getRotation().getDegrees(),
                desiredHeading.getDegrees()
        );
        vtFeedback*=(Math.PI/180);

//        RobotLog.vv(this.getClass().getSimpleName(), "Calculate parameters : (trajectory pose: %s desired vel: %s desired heading: %s)", trajectoryPose, desiredLinearVelocityMetersPerSecond, desiredHeading);
//        RobotLog.vv(this.getClass().getSimpleName(), "Calculate outputs    : (vxTarget: %s vyTarget: %s vt feedback: %s vx feedback: %s vy feedback: %s)", vxTarget, vyTarget, vtFeedback, vxFeedback, vyFeedback);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                vxTarget - vxFeedback, vyTarget - vyFeedback, vtFeedback, currentPose.getRotation()
        );
    }

    public ChassisSpeeds calculate(
            Pose2d currentPose, Trajectory.State desiredState, Rotation2d desiredHeading) {
        return calculate(
                currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, desiredHeading);
    }
}
