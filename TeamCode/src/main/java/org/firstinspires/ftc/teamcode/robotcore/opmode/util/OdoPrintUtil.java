package org.firstinspires.ftc.teamcode.robotcore.opmode.util;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.OdoWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

@TeleOp(group = "2")
@Config
public class
OdoPrintUtil extends LinearOpMode {
    private static final String TAG = "ODOM";

    @Override
    public void runOpMode() throws InterruptedException {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            OdoWheelSpeeds odoWheelSpeeds = chassisSubsystem.getOdoWheelSpeeds();

            RobotLog.dd(TAG, "left     : %s", odoWheelSpeeds.leftMetersPerSecond);
            RobotLog.dd(TAG, "right    : %s", odoWheelSpeeds.rightMetersPerSecond);
            RobotLog.dd(TAG, "center    : %s", odoWheelSpeeds.centerMetersPerSecond);

            telemetry.addData("left speed", odoWheelSpeeds.leftMetersPerSecond);
            telemetry.addData("right speed", odoWheelSpeeds.rightMetersPerSecond);
            telemetry.addData("center speed", odoWheelSpeeds.centerMetersPerSecond);

            telemetry.addData("left position", chassisSubsystem.getLeftOdometer().getPosition());
            telemetry.addData("right position", chassisSubsystem.getRightOdometer().getPosition());
            telemetry.addData("center position", chassisSubsystem.getCenterOdometer().getPosition());

            chassisSubsystem.periodic();
            telemetry.update();
        }
    }
}
