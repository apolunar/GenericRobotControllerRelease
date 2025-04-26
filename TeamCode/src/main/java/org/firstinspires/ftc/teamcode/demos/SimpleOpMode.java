package org.firstinspires.ftc.teamcode.demos;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class SimpleOpMode extends OpMode {
    private CRServo servo;
    private RevBlinkinLedDriver blinkin;
//    private Motor armMotor2;

    GamepadEx gamepadEx;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "ton");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        //        armMotor2 = new Motor(hardwareMap, "slideRight");
//        armMotor2 = new Motor(hardwareMap, "screw");

//        armMotor1.setRunMode(Motor.RunMode.VelocityControl);
//        armMotor2.setRunMode(Motor.RunMode.VelocityControl);

//        gamepadEx = new GamepadEx(gamepad1);
        // IllegalArgumentException
    }

    @Override
    public void loop() {
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        servo.setPower(1);
    }
}
