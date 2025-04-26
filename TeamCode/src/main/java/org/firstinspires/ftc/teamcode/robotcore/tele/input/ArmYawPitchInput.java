package org.firstinspires.ftc.teamcode.robotcore.tele.input;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.tele.Actions;

public class ArmYawPitchInput extends CommandBase {
    public static double SIDE_DEPOSIT_EXT_POWER = 0.75;

    private final ExtensionSubsystem extensionSubsystem;
    private final LiftSubsystem liftSubsystem;
    private final Telemetry telemetry;
    private final Actions actions;

    private final ElapsedTime sampleTransferTimer = new ElapsedTime();;

    public ArmYawPitchInput(ExtensionSubsystem extensionSubsystem, LiftSubsystem liftSubsystem, Telemetry telemetry, Actions actions) {
        this.extensionSubsystem = extensionSubsystem;
        this.liftSubsystem = liftSubsystem;
        this.telemetry = telemetry;
        this.actions = actions;

        addRequirements(extensionSubsystem);
    }

    @Override
    public void execute() {
        // Not center yaw or side deposit
        if (!actions.centerYaw && !actions.sideDeposit) {
            extensionSubsystem.driveYaw(-actions.yaw);
        } // Normal operation of yaw

        if (actions.sampleTransferButton.wasJustPressed()) {
            sampleTransferTimer.reset();
        }

        if (actions.prepareIntake) {
            extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.MID);
            extensionSubsystem.setClawPosition(ExtensionSubsystem.ClawPosition.OPEN);
        } // Prepare intake
        else if (actions.sideDeposit) {
            if (!extensionSubsystem.getLimit()) {
                extensionSubsystem.setExtensionPower(SIDE_DEPOSIT_EXT_POWER);
            }
            extensionSubsystem.setArmYawVoltage(ExtensionSubsystem.SIDE_DEPOSIT);
        } // Side deposit
        else if (actions.centerYaw) {
            extensionSubsystem.setArmYawVoltage(ExtensionSubsystem.CENTER_YAW);
        } // Center yaw
        else if (actions.pitchHigh) {
            extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.HIGH);
        } // Pitch high
        else if (actions.sampleTransferButton.pressed) {
            // Sample transfer
            if (sampleTransferTimer.seconds() > 2) {
                liftSubsystem.setSwingPosition(LiftSubsystem.SwingPosition.BASKET);
//                targetLiftPosition = liftSubsystem.getLiftPosition();
            } else if (sampleTransferTimer.seconds() > 1.5) {
                liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);
                extensionSubsystem.setClawPosition(ExtensionSubsystem.ClawPosition.OPEN);
            } else if (sampleTransferTimer.seconds() > 0.5) {
                liftSubsystem.setLiftPosition(LiftSubsystem.SAMPLE_TRANSFER_POSITION);
            } else {
                extensionSubsystem.setArmYawVoltage(ExtensionSubsystem.CENTER_YAW);
                extensionSubsystem.setArmClawPitchSync(ExtensionSubsystem.Pitch.TRANSFER);
                extensionSubsystem.setExtensionPosition(ExtensionSubsystem.SAMPLE_TRANSFER_POSITION);
                liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.OPEN);
                liftSubsystem.setSwingPosition(LiftSubsystem.SwingPosition.DELIVERY);
            }
        }
    }
}
