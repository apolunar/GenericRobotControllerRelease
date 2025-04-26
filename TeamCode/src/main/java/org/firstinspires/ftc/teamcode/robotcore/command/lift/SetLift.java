package org.firstinspires.ftc.teamcode.robotcore.command.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.math.util.Range;
import org.firstinspires.ftc.teamcode.robotcore.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;

/**
 * Moves list at target velocity.
 * Additionally moves swing to intake position if slide is below -700 or delivery if it's above.
 */
public class SetLift extends CommandBase {
    private final LiftSubsystem liftSubsystem;

    private final int position;
    private final int targetPower;
    private final double targetErrorPercent;
    private final boolean setSwing;

    // Error integral
    private static final double eI = 100;
    private double positionDelta = 1;
    private ElapsedTime runtime;
    private boolean firstRun = true;

    public SetLift(LiftSubsystem liftSubsystem, int position, int targetPower, double targetErrorPercent, boolean setSwing) {
        this.liftSubsystem = liftSubsystem;
        this.position = position;
        this.targetPower = targetPower;
        this.targetErrorPercent = targetErrorPercent;
        this.setSwing = setSwing;
    }

    public SetLift(LiftSubsystem liftSubsystem, int position, int targetPower) {
        this(liftSubsystem, position, targetPower, AutoConfig.LIFT_SUBSYSTEM_ERROR, true);
    }

    public SetLift(LiftSubsystem liftSubsystem, int position, boolean setSwing) {
        this(liftSubsystem, position, 2000, AutoConfig.LIFT_SUBSYSTEM_ERROR, setSwing);
    }

    public SetLift(LiftSubsystem liftSubsystem, int position) {
        this(liftSubsystem, position, 2000);
    }

    @Override
    public void execute() {
        if (firstRun) {
            runtime = new ElapsedTime();
            positionDelta = Math.abs(liftSubsystem.getLiftPosition() - position);
            firstRun = false;
        }

        liftSubsystem.setLiftPosition(position, targetPower);

        if (setSwing) {
            liftSubsystem.setSwingPosition(
                    position < LiftSubsystem.SHOULDSWING_POSITION && liftSubsystem.getLiftPosition() < LiftSubsystem.SHOULDSWING_POSITION ?
                            LiftSubsystem.SwingPosition.DELIVERY : LiftSubsystem.SwingPosition.INTAKE
            );
        }
    }

    @Override
    public boolean isFinished() {
        RobotLog.dd(this.m_name, "Pos delta I: %s", runtime.seconds() / positionDelta * eI);
        boolean complete = Range.valueWithinPercentage(liftSubsystem.getLiftPosition(), position, targetErrorPercent + runtime.seconds() / positionDelta * eI);
        if (complete) {
            RobotLog.dd(this.m_name, "Lift reached target...");
        }
        return complete;
    }

    @Override
    public void end(boolean interrupted) {
        liftSubsystem.setLiftPower(0);
    }
}
