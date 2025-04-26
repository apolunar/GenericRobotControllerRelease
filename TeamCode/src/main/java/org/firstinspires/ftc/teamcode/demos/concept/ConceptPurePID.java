package org.firstinspires.ftc.teamcode.demos.concept;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

import lombok.RequiredArgsConstructor;

@TeleOp(name = "!DDDDDDDDDDDDDDd")
@Disabled
public class ConceptPurePID extends CommandOpMode {
    @Override
    public void initialize() {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);

        telemetry.log().add("Latest pose : %s", chassisSubsystem.getLatestPose());

        PurePIDF purePIDF = new PurePIDF(
                chassisSubsystem,
                new Pose2d(0, 12, new Rotation2d(0)),
                new PIDFController(1, 0, 0, 1),
                new PIDFController(1, 0, 0, 1)
        );

        schedule(purePIDF);
        register(chassisSubsystem);
    }
}

/**
 * <a href="https://docs.ftclib.org/ftclib/features/controllers">...</a>
 */
@RequiredArgsConstructor
class PurePIDF extends CommandBase {
    private final ChassisSubsystem chassisSubsystem;
    private final Pose2d target;
    private final PIDFController xController, yController;

    @Override
    public void initialize() {
        xController.setSetPoint(target.getX());
        yController.setSetPoint(target.getY());
    }

    @Override
    public void execute() {
        Pose2d curPose = chassisSubsystem.getLatestPose();

        double xOut = xController.calculate(curPose.getX());
        double yOut = yController.calculate(curPose.getY());

        RobotLog.dd(this.m_name, "Pose : %s", curPose);
        RobotLog.dd(this.m_name, "x : %s", xOut);
        RobotLog.dd(this.m_name, "y : %s", yOut);

        // meters per second and rotation per second
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xOut, yOut, 0);
        MecanumDriveWheelSpeeds mecanumDriveWheelSpeeds = chassisSubsystem.getDriveKinematics().toWheelSpeeds(chassisSpeeds);

        RobotLog.dd(this.m_name, "Wheel speeds fL fR bL bR : %s %s %s %s",
                mecanumDriveWheelSpeeds.frontLeftMetersPerSecond,
                mecanumDriveWheelSpeeds.frontRightMetersPerSecond,
                mecanumDriveWheelSpeeds.rearLeftMetersPerSecond,
                mecanumDriveWheelSpeeds.rearRightMetersPerSecond
        );

        chassisSubsystem.drive(xOut, yOut, 0);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetPoint() && yController.atSetPoint();
    }
}