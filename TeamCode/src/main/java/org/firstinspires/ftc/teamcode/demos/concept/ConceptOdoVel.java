package org.firstinspires.ftc.teamcode.demos.concept;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.OdoWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

@TeleOp(group = "2")
public class ConceptOdoVel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(chassisSubsystem);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            Pose2d latestPose = chassisSubsystem.getLatestPose();

            RobotLog.dd(this.getClass().getSimpleName(), "latest pose : %s", latestPose);

            OdoWheelSpeeds wheelSpeeds = chassisSubsystem.getOdoWheelSpeeds();

            RobotLog.dd(this.getClass().getSimpleName(), "left : %s", wheelSpeeds.leftMetersPerSecond);
            RobotLog.dd(this.getClass().getSimpleName(), "right : %s", wheelSpeeds.rightMetersPerSecond);
            RobotLog.dd(this.getClass().getSimpleName(), "parallel : %s", wheelSpeeds.centerMetersPerSecond);

            double spinSpeed = wheelSpeeds.rightMetersPerSecond + wheelSpeeds.leftMetersPerSecond;

            RobotLog.dd(this.getClass().getSimpleName(), "turn speed in/s : %s", spinSpeed);
            RobotLog.dd(this.getClass().getSimpleName(), "turn speed rad/s : %s", spinSpeed / (ChassisSubsystem.TRACK_WIDTH/2));

            chassisSubsystem.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

//            ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
//                    gamepad1.left_stick_x*ChassisSubsystem.MAX_X_LINEAR_VELOCITY,
//                    gamepad1.left_stick_y*ChassisSubsystem.MAX_X_LINEAR_VELOCITY,
//                    gamepad1.right_stick_x*10
//            );
//
//            chassisSubsystem.driveChassisSpeeds(chassisSpeeds);
//            chassisSubsystem.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}