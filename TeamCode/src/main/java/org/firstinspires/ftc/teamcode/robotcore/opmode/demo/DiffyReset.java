package org.firstinspires.ftc.teamcode.robotcore.opmode.demo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(group = "2")
public class DiffyReset extends OpMode {
    Servo diffyL;
    Servo diffyR;

    @Override
    public void init() {
        diffyL = hardwareMap.get(Servo.class,"diffy l");
        diffyR = hardwareMap.get(Servo.class,"diffy r");
    }

    @Override
    public void loop() {
        diffyL.setPosition(0.5);
        diffyR.setPosition(0.5);
    }
}
