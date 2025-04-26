package org.firstinspires.ftc.teamcode.robotcore.opmode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.logging.LogWrapper;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.tele.Actions;
import org.firstinspires.ftc.teamcode.robotcore.tele.input.ArmPickupInput;
import org.firstinspires.ftc.teamcode.robotcore.tele.input.ArmYawPitchInput;
import org.firstinspires.ftc.teamcode.robotcore.tele.input.ExtensionInput;

import java.util.List;

@TeleOp(group = "1")
@Config
@Disabled
public class AdaptiveTeleOpMode extends LinearOpMode {
    private static double INITIAL_SWING_POSITION = 0.85;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        LogWrapper.init(telemetry);


        Actions actions = new Actions(gamepad1, gamepad2);

        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem(hardwareMap);
        LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap);

        CommandScheduler.getInstance().registerSubsystem(
                chassisSubsystem, extensionSubsystem, liftSubsystem
        );

        extensionSubsystem.setDefaultCommand(
                new ParallelCommandGroup(
                    new ExtensionInput(extensionSubsystem, telemetry, actions),
                    new ArmPickupInput(extensionSubsystem, liftSubsystem, telemetry, actions),
                    new ArmYawPitchInput(extensionSubsystem, liftSubsystem, telemetry, actions)
                )
        );

        liftSubsystem.setSwing(INITIAL_SWING_POSITION);
        liftSubsystem.setSwingPosition(LiftSubsystem.SwingPosition.DELIVERY);
        extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.TRANSFER);
        liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.OPEN);

        while (opModeInInit() && !isStopRequested()) {
            telemetry.addLine("Waiting for start...");
        }

        while (opModeIsActive() && !isStopRequested()) {
            // One bulk read per cycle
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            actions.updateActions();
            CommandScheduler.getInstance().run();
            telemetry.update();
        }
    }
}
