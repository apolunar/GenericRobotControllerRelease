package org.firstinspires.ftc.teamcode.robotcore.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;

import lombok.RequiredArgsConstructor;

/**
 * Completes instantly.
 */
@RequiredArgsConstructor
public class SetLiftClaw extends CommandBase {
    private final LiftSubsystem liftSubsystem;

    private final LiftSubsystem.ClawPosition clawPosition;

    @Override
    public void execute() {
        liftSubsystem.setClawPosition(clawPosition);
    }

    @Override
    public boolean isFinished() {
        RobotLog.dd(this.m_name, "Set claw to : %s", clawPosition);
        return true;
    }
}
