package org.firstinspires.ftc.teamcode.demos.concept;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

@TeleOp(name = "!Forward Demo")
@Disabled
public class ConceptForward extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(chassisSubsystem);

        double targetX = -12;
        double targetY = 12;
        double targetT = 0;
        double p = 0.1;
        double tol = .1;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            Pose2d latestPose = chassisSubsystem.getLatestPose();

            double xErr = targetX - latestPose.getX();
            double yErr = targetY - latestPose.getY();
            double tErr = targetT - Math.toDegrees(latestPose.getHeading());

            RobotLog.dd(this.getClass().getSimpleName(), "xErr : %s", xErr);
            RobotLog.dd(this.getClass().getSimpleName(), "yErr : %s", yErr);
            RobotLog.dd(this.getClass().getSimpleName(), "tErr : %s", tErr);

            if (!(Math.abs(xErr) < tol && Math.abs(yErr) < tol && Math.abs(tErr) < tol)) {
                chassisSubsystem.drive(-p * yErr, p * xErr, -p * tErr);
            }
            else {
                chassisSubsystem.drive(0, 0, 0);
            }
        }
    }
}