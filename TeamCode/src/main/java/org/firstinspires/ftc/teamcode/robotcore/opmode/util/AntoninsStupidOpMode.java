package org.firstinspires.ftc.teamcode.robotcore.opmode.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class AntoninsStupidOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motor1 = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor motor2 = hardwareMap.get(DcMotor.class, "motor2");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            motor1.setPower(gamepad1.left_stick_y);
            motor2.setPower(gamepad1.right_stick_y);
        }
    }
}
