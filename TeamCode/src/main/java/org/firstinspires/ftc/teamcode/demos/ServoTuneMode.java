package org.firstinspires.ftc.teamcode.demos;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class ServoTuneMode extends OpMode {
    private Servo spatL;
    private Servo spatR;

    double spatLPos;
    double spatRPos;

    GamepadEx gamepadEx;

    @Override
    public void init() {
        spatL = hardwareMap.get(Servo.class, "spatL");
        spatR = hardwareMap.get(Servo.class, "spatR");

        gamepadEx = new GamepadEx(gamepad1);
        // IllegalArgumentException
    }

    @Override
    public void loop() {
        gamepadEx.readButtons();

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.X)) {
            spatLPos -= 0.01;
        }
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.B)) {
            spatLPos += 0.01;
        }

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) {
            spatRPos += 0.01;
        }
        if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
            spatRPos -= 0.01;
        }

        if (gamepadEx.wasJustPressed(GamepadKeys.Button.START)) {
            spatL.setPosition(spatLPos);
            spatR.setPosition(spatRPos);
        }

        telemetry.addData("Spat L position", spatLPos);
        telemetry.addData("Spat R position", spatRPos);


    }
}
