package org.firstinspires.ftc.teamcode.robotcore.tele.input;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.tele.Actions;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class MovementInput extends CommandBase {
    public static final double MIN_DRIVE_SPEED = 0.2;
    public static final double SOMEWHAT_FASTER_DRIVE_SPEED = 0.5;
    public static final double DEFAULT_DRIVE_SPEED = 1;

    private final ChassisSubsystem chassisSubsystem;
    private final ExtensionSubsystem extensionSubsystem;
    private final ExtensionInput extensionInput;
    private final Actions actions;

    @Override
    public void execute() {
        double speedMod = Math.max(MIN_DRIVE_SPEED, DEFAULT_DRIVE_SPEED - (actions.somewhatFasterSpeedModifier * (1 - SOMEWHAT_FASTER_DRIVE_SPEED)) - (actions.slowSpeedModifier * (1 - MIN_DRIVE_SPEED)));
        double powerX = -actions.powerX * speedMod;
        double powerY = actions.powerY * speedMod;
        double powerRotate = actions.powerRotate * speedMod;

        if (extensionSubsystem.getLimit()) {
            if (actions.extension > 0) {
                powerY += actions.extension;
            } // Drive backwards
        }
        else if (extensionSubsystem.getExtensionPosition() >= extensionInput.getVariableExtLimit()) {
            if (actions.extension < 0) {
                powerY -= actions.extension;
            } // Drive forwards
        }

        chassisSubsystem.drive(
                powerX,
                powerY,
                powerRotate);
    }
}
