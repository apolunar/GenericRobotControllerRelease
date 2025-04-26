package org.firstinspires.ftc.teamcode.robotcore.tele.input;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.LiftSubsystem;
import org.firstinspires.ftc.teamcode.robotcore.tele.Actions;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class LiftInput extends CommandBase {
    private final LiftSubsystem liftSubsystem;
    private final ExtensionSubsystem extensionSubsystem;
    private final Telemetry telemetry;
    private final Actions actions;

    private static final double MAX_LIFT_CURRENT = 10;
    private static final double LIFT_UP_POWER = 0.5;
    private static final double LIFT_DOWN_POWER = 1;

    private final ElapsedTime armRuntime = new ElapsedTime();
    private ElapsedTime clawSwingTimer = new ElapsedTime();
    private boolean deliveryClawClosed = false;
    private int targetLiftPosition = 0;
    private boolean swingSwung = false;

    @Override
    public void execute() {
        telemetry.addData("Lift position", liftSubsystem.getLiftPosition());
        telemetry.addData("Lift limit", liftSubsystem.getLimit());

        deliveryClawClosed = actions.toggleDeliveryClaw.isPressed();

        if (actions.specDeliveryPosition) {
            extensionSubsystem.setClawPosition(ExtensionSubsystem.ClawPosition.OPEN);
            liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);
            if (armRuntime.milliseconds() > 600) {
                targetLiftPosition = LiftSubsystem.DELIVERY_POSITION;
                if (liftSubsystem.getLiftPosition() < LiftSubsystem.SHOULDSWING_POSITION) {
                    liftSubsystem.setSwingPosition(LiftSubsystem.SwingPosition.DELIVERY);
                    swingSwung = true;
                }
            }
        } // Delivery position
        else if (actions.subWithdraw) {
            targetLiftPosition = LiftSubsystem.WITHDRAW_POSITION;
        } // Withdraw position
        else if (actions.specIntake) {
            targetLiftPosition = LiftSubsystem.INTAKE_POSITION;
            if (liftSubsystem.getLiftPosition() >= LiftSubsystem.SHOULDSWING_POSITION) {
//                if (!justSwung) {
//                    justSwung = true;
//                    clawSwingTimer.reset();
//                }
                liftSubsystem.setSwingPosition(LiftSubsystem.SwingPosition.INTAKE);
                if (clawSwingTimer.seconds() > .15) {
                    liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.OPEN);
                    deliveryClawClosed = false;
                } else {
                    liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);
                    deliveryClawClosed = true;
                }
            } else {
                liftSubsystem.setClawPosition(LiftSubsystem.ClawPosition.CLOSED);
//                justSwung = false;
                deliveryClawClosed = true;
            }
            swingSwung = false;
        }

        if (actions.liftUp || actions.liftDown) { // Drives normally if not using presets
            if (liftSubsystem.getLimit()) { // Resets slide position if limit switch is pressed
                liftSubsystem.zeroLiftPosition();
                liftSubsystem.setLiftPower(actions.liftUp ? LIFT_UP_POWER : 0); // Only go up
            }
            else if (liftSubsystem.getLiftCurrent() < MAX_LIFT_CURRENT) {
                if (actions.liftUp) {
                    liftSubsystem.setLiftPower(LIFT_UP_POWER);
                } else if (actions.liftDown) {
                    liftSubsystem.setLiftPower(-LIFT_DOWN_POWER);
                } else {
                    liftSubsystem.setLiftPower(0);
                }
            } // Prevent overcurrent from hang or something
            else {
                liftSubsystem.setLiftPower(0);
            }
            targetLiftPosition = liftSubsystem.getLiftPosition();
        } else if (!actions.sideDeposit && !actions.ascendToRobotHeaven) {
            liftSubsystem.setLiftPosition(targetLiftPosition);
        }


        // Swing control
        if (actions.swing.wasJustPressed()) {
            swingSwung = !swingSwung;
            liftSubsystem.setSwingPosition(swingSwung ? LiftSubsystem.SwingPosition.DELIVERY : LiftSubsystem.SwingPosition.INTAKE);
        }
    }
}
