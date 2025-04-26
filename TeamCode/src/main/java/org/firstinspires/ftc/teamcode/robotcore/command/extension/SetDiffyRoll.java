package org.firstinspires.ftc.teamcode.robotcore.command.extension;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;

import lombok.RequiredArgsConstructor;

/**
 * Completes instantly.
 */
@RequiredArgsConstructor
public class SetDiffyRoll extends CommandBase {
    private final ExtensionSubsystem extensionSubsystem;

    private final double clawRoll;

    @Override
    public void execute() {
        extensionSubsystem.setClawRoll(clawRoll);
    }

    @Override
    public boolean isFinished() {
        RobotLog.dd(this.m_name, "Set claw to : %s", clawRoll);
        return true;
    }
}
