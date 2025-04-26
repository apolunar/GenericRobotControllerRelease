package org.firstinspires.ftc.teamcode.robotcore.opmode.util.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "2")
public class DiffyFineTuner extends OpMode {
    Servo servoL;
    Servo servoR;
    double diffyPosition = 0.5;
    double diffyAngle = 0.5;
    boolean posChanged;
    boolean angChanged;
    @Override
    public void init() {
        servoL = hardwareMap.get(Servo.class,"servo l");
        servoR = hardwareMap.get(Servo.class,"servo r");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            double lPower = (diffyPosition+(diffyAngle))/2;
            double rPower = (diffyPosition+(1-diffyAngle))/2;
            telemetry.addData("Left Servo: ",lPower);
            telemetry.addData("Right Servo: ",rPower);
            servoL.setPosition(lPower);
            servoR.setPosition(rPower);
        }
        if (!posChanged) {
            if (gamepad1.left_bumper) {
                diffyPosition += 0.01;
                posChanged = true;
            } else if (gamepad1.left_trigger > 0) {
                diffyPosition -= 0.01;
                posChanged = true;
            }
            if (gamepad1.dpad_right) {
                diffyPosition += 0.1;
                posChanged = true;
            } else if (gamepad1.dpad_left) {
                diffyPosition -= 0.1;
                posChanged = true;
            }
        } else if (!gamepad1.dpad_left && !gamepad1.dpad_right &&
        !gamepad1.left_bumper && gamepad1.left_trigger == 0) {
            posChanged = false;
        }
        telemetry.addData("Diffy Position: ",diffyPosition);

        if (!angChanged) {
            if (gamepad1.right_bumper) {
                diffyAngle += 0.01;
                angChanged = true;
            } else if (gamepad1.right_trigger > 0) {
                diffyAngle -= 0.01;
                angChanged = true;
            }
            if (gamepad1.b) {
                diffyAngle += 0.1;
                angChanged = true;
            } else if (gamepad1.x) {
                diffyAngle -= 0.1;
                angChanged = true;
            }
        } else if (!gamepad1.b && !gamepad1.x &&
                !gamepad1.right_bumper && gamepad1.right_trigger == 0) {
            angChanged = false;
        }
        telemetry.addData("Diffy Angle: ",diffyAngle);
        telemetry.addData("Joystick Angle: ",(Math.atan(gamepad1.right_stick_x/gamepad1.right_stick_y)/(Math.PI))+0.5);
    }
}
