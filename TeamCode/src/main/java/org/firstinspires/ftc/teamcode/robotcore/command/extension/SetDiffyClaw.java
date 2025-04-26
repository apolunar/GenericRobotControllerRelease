package org.firstinspires.ftc.teamcode.robotcore.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;

import lombok.RequiredArgsConstructor;

/**
 * Completes instantly.
 */
@RequiredArgsConstructor
public class SetDiffyClaw extends CommandBase {
    private final ExtensionSubsystem extensionSubsystem;

    private final ExtensionSubsystem.ClawPosition clawPosition;

    @Override
    public void execute() {
        extensionSubsystem.setClawPosition(clawPosition);
    }

    @Override
    public boolean isFinished() {
        RobotLog.dd(this.m_name, "Set claw position to : %s", clawPosition);
        // TODO: There's supposedly feedback on the claw...
        return true;
    }
}
