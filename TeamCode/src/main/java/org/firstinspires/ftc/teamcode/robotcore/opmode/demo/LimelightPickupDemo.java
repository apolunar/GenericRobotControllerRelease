package org.firstinspires.ftc.teamcode.robotcore.opmode.demo;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcore.command.extension.PickupSample;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LimelightSubsystem;

@TeleOp(group = "2")
public class LimelightPickupDemo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(hardwareMap);
        CameraSubsystem cameraSubsystem = new CameraSubsystem(hardwareMap);
        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(hardwareMap);

        PickupSample pickupSample = new PickupSample(extensionSubsystem, cameraSubsystem, limelightSubsystem, telemetry);
        pickupSample.schedule();



        extensionSubsystem.setArmPitch(ExtensionSubsystem.ArmPitch.MID);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            telemetry.update();
        }
    }
}
