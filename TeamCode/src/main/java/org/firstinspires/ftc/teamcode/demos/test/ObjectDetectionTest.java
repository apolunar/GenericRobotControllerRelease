package org.firstinspires.ftc.teamcode.demos.test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

@TeleOp(group = "Testing")
public class ObjectDetectionTest extends OpMode {
    CameraSubsystem cameraSubsystem;

    @Override
    public void init() {
        cameraSubsystem = new CameraSubsystem(hardwareMap);

        cameraSubsystem.register();
    }

    int leftOffset;
    int rightOffset;

    @Override
    public void loop() {
        telemetry.addLine("Running");

        CommandScheduler.getInstance().run();


    }
}
