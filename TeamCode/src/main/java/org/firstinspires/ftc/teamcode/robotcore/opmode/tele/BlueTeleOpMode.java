package org.firstinspires.ftc.teamcode.robotcore.opmode.tele;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcore.game.Alliance;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;

@TeleOp
public class BlueTeleOpMode extends CommandOpMode {
    @Override
    public void initialize() {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);
        ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(hardwareMap);
        CameraSubsystem cameraSubsystem = new CameraSubsystem(hardwareMap);

        schedule(
                new TeleOpMode(gamepad1, gamepad2, chassisSubsystem, liftSubsystem, extensionSubsystem, cameraSubsystem, telemetry, Alliance.BLUE_ALLIANCE)
        );
    }
}
