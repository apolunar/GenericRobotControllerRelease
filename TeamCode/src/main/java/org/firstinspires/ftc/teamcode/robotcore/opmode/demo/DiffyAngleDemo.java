package org.firstinspires.ftc.teamcode.robotcore.opmode.demo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;

@TeleOp(group = "2")
public class DiffyAngleDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            extensionSubsystem.setClawRoll(0.6);
        }
    }
}
