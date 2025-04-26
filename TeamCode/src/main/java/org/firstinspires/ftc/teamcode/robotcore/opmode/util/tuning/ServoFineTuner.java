package org.firstinspires.ftc.teamcode.robotcore.opmode.util.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(group = "2")
public class ServoFineTuner extends OpMode {
    ServoImplEx servo;
    double servoPosition = 0;
    boolean buttonPressed;
    @Override
    public void init() {
        servo = hardwareMap.get(ServoImplEx.class,"servo");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo.setPosition(servoPosition);
        } else {
            servo.setPwmDisable();
        }
        if (!buttonPressed) {
            if (gamepad1.right_bumper) {
                servoPosition += 0.01;
                buttonPressed = true;
            } else if (gamepad1.left_bumper) {
                servoPosition -= 0.01;
                buttonPressed = true;
            }
            if (gamepad1.dpad_right) {
                servoPosition += 0.1;
                buttonPressed = true;
            } else if (gamepad1.dpad_left) {
                servoPosition -= 0.1;
                buttonPressed = true;
            }
        } else if (!gamepad1.dpad_left && !gamepad1.dpad_right &&
        !gamepad1.left_bumper && !gamepad1.right_bumper) {
            buttonPressed = false;
        }
        telemetry.addData("Servo Position: ",servoPosition);
    }
}
