package org.firstinspires.ftc.teamcode.robotcore.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;

import lombok.RequiredArgsConstructor;

/**
 * Completes instantly.
 */
@RequiredArgsConstructor
public class MoveSwing extends CommandBase {
    private final LiftSubsystem liftSubsystem;

    private final LiftSubsystem.SwingPosition swing;

    @Override
    public void execute() {
        liftSubsystem.setSwingPosition(swing);
    }

    @Override
    public boolean isFinished() {
        RobotLog.dd(this.m_name, "Set swing to : %s", swing);
        return true;
    }
}
