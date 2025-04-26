package org.firstinspires.ftc.teamcode.robotcore.command.chassis;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class DriveToPoint extends CommandBase {
    private final ChassisSubsystem chassisSubsystem;
    private final Pose2d target;
    private static final double TOLERANCE    = 0.5;
    private static final double PROPORTIONAL = 0.8;

    private double xErr;
    private double yErr;

    @Override
    public void initialize() {
        RobotLog.dd(this.m_name, "Start drive to point with target : %s", target);
    }

    @Override
    public void execute() {
        Pose2d latestPose = chassisSubsystem.getLatestPose();

        xErr = target.getX() - latestPose.getX();
        yErr = target.getY() - latestPose.getY();
        // tErr = latestPose.getRotation().getDegrees()-target.getRotation().getDegrees();

        RobotLog.dd(this.getClass().getSimpleName(), "xErr : %s", xErr);
        RobotLog.dd(this.getClass().getSimpleName(), "yErr : %s", yErr);

        // NOTE: Cannot drive and turn, or turn in general with this controller
        chassisSubsystem.driveFieldCentric(
                PROPORTIONAL*-yErr,
                PROPORTIONAL*xErr,
                0, latestPose.getHeading()
        );
    }

    @Override
    public void end(boolean good) {
        chassisSubsystem.drive(0, 0, 0);
    }

    private boolean atXY() {
        return Math.abs(xErr) < TOLERANCE && Math.abs(yErr) < TOLERANCE;
    }

    @Override
    public boolean isFinished() {
        return atXY();
    }
}
