package org.firstinspires.ftc.teamcode.robotcore.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class NoAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("THIS OP MODE DOES NOTHING");
        telemetry.addLine("IF YOU'RE RUNNING THIS SOMETHING HAS GONE VERY WRONG");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addLine(":(");
            telemetry.update();
        }
    }
}
