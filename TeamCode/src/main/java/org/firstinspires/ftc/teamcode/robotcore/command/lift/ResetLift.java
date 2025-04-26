package org.firstinspires.ftc.teamcode.robotcore.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class ResetLift extends CommandBase {
    private final LiftSubsystem liftSubsystem;

    @Override
    public void execute() {
        double position = liftSubsystem.getLiftPosition();
        RobotLog.dd(this.m_name, "Lift position : %s", position);
        liftSubsystem.setLiftPower(.5);
    }

    @Override
    public boolean isFinished() {
        if (liftSubsystem.getLimit()) {
            liftSubsystem.zeroLiftPosition();
            return true;
        }
        return false;
    }
}
