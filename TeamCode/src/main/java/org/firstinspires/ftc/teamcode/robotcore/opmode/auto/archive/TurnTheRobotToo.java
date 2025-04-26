package org.firstinspires.ftc.teamcode.robotcore.opmode.auto.archive;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

@Autonomous(group = "Main")
@Disabled
public class TurnTheRobotToo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);

        double targetT = 90;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            Pose2d latestPose = chassisSubsystem.getLatestPose();
            double tErr = Math.toDegrees(latestPose.getHeading())-targetT;

            RobotLog.dd(this.getClass().getSimpleName(), "tErr : %s", tErr);

            if (Math.abs(tErr) > 1) {
                chassisSubsystem.drive(0, 0, tErr/ targetT);
            }
            else {
                chassisSubsystem.drive(0, 0, 0);
            }
        }
    }
}
