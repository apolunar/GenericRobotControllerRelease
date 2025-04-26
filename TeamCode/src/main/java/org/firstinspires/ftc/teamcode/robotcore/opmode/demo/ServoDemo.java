package org.firstinspires.ftc.teamcode.robotcore.opmode.demo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class ServoDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo serv1 = hardwareMap.get(Servo.class, "servo1");
        Servo serv2 = hardwareMap.get(Servo.class, "servo2");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            serv1.setPosition(0);
            serv2.setPosition(0);
        }
    }
}
