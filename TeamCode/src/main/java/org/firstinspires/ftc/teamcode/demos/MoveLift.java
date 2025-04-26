package org.firstinspires.ftc.teamcode.demos;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.math.util.Range;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class MoveLift extends CommandBase {
    private final ExtendSubsystem extendSubsystem;

    private final int targetTicks;
    private final int targetVelocity;

    @Override
    public void initialize() {
        extendSubsystem.setTargetTicks(targetTicks, targetVelocity, true);
    }

    @Override
    public boolean isFinished() {
        return Range.valueWithinPercentage(extendSubsystem.getSlideTicks(), targetTicks, .05);
    }
}
