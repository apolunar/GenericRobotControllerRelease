package org.firstinspires.ftc.teamcode.robotcore.tele.input;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.tele.Actions;

import lombok.Getter;

public class ExtensionInput extends CommandBase {
    public static double MAX_EXT_VEL_TPS = 1590; // TODO

    private final ExtensionSubsystem extensionSubsystem;
    private final Telemetry telemetry;
    private final Actions actions;

    @Getter
    private int variableExtLimit = 0;

    public ExtensionInput(ExtensionSubsystem extensionSubsystem, Telemetry telemetry, Actions actions) {
        this.extensionSubsystem = extensionSubsystem;
        this.telemetry = telemetry;
        this.actions = actions;

        addRequirements(extensionSubsystem);
    }

    @Override
    public void execute() {
        int extensionPosition = extensionSubsystem.getExtensionPosition();
        double extensionPercentage = (double) extensionPosition / ExtensionSubsystem.MAX_EXT_POSITION;

        telemetry.addData("Extension percentage", extensionPercentage * 100);
        telemetry.addData("Extension limit switch", extensionSubsystem.getLimit());

        variableExtLimit = (int) (ExtensionSubsystem.EXT_LIMIT_POSITION -
                (ExtensionSubsystem.INTAKE_LENGTH * Math.cos(extensionSubsystem.getZeroedYawRadians())));

        telemetry.addData("Extension variable limit", variableExtLimit);

        if (!actions.sideDeposit) {
            if (extensionSubsystem.getLimit()) {
                extensionSubsystem.zeroSlidePosition();

                if (actions.extension < 0) {
                    extensionSubsystem.setExtensionPower(-actions.extension);
                }
            }
            else if (extensionSubsystem.getExtensionPosition() >= variableExtLimit && actions.extension > 0) {
                extensionSubsystem.setExtensionPower(-actions.extension);
            } // Inwards motion
        } // Not side deposit
    }
}
