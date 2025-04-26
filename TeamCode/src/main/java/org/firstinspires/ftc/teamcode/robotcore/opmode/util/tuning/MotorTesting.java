package org.firstinspires.ftc.teamcode.robotcore.opmode.util.tuning;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp
public class MotorTesting extends OpMode {
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;


    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class,"intakeSlide");
        motor2 = hardwareMap.get(DcMotor.class,"armSlide");
    }

    @Override
    public void loop() {
        motor1.setPower(gamepad1.left_stick_y);
        motor2.setPower(gamepad1.right_stick_y);
    }
}
