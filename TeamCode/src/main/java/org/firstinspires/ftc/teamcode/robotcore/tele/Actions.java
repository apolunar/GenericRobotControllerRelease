package org.firstinspires.ftc.teamcode.robotcore.tele;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.gamepad.MyGamepadButton;
import org.firstinspires.ftc.teamcode.gamepad.MyToggleButtonReader;

public class Actions {
    public final Gamepad gamepad1;
    public final Gamepad gamepad2;

    public double slowSpeedModifier, somewhatFasterSpeedModifier;
    public double powerX, powerY, powerRotate;
    public final MyToggleButtonReader toggleDeliveryClaw = new MyToggleButtonReader();
    public boolean subWithdraw;
    public boolean specIntake;
    private final MyToggleButtonReader ascendToggle = new MyToggleButtonReader();
    public boolean ascendToRobotHeaven;

    public double extension;
    public boolean sideDeposit, centerYaw;
    public double yaw;
    public double diffyX, diffyY;
    public boolean prepareIntake;
    public boolean pitchHigh;
    public final MyGamepadButton sampleTransferButton = new MyGamepadButton();
    public final MyGamepadButton intakeButton = new MyGamepadButton();
    public boolean release;
    public boolean specDeliveryPosition;
    public boolean liftUp, liftDown;
    public MyGamepadButton swing = new MyGamepadButton();

    public Actions(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    public void updateActions() {
        // Gamepad 1
        slowSpeedModifier = DynamicInput.getAxisValue(gamepad1, GamepadSettings.gamepad1ButtonMapping.slowSpeedModifier);
        somewhatFasterSpeedModifier = DynamicInput.getAxisValue(gamepad1, GamepadSettings.gamepad1ButtonMapping.somewhatFasterSpeedModifier);
        powerX = GamepadSettings.applyBoostCurve(GamepadSettings.applyDeadzone(DynamicInput.getAxisValue(gamepad1, GamepadSettings.gamepad1ButtonMapping.powerX)));
        powerY = GamepadSettings.applyBoostCurve(GamepadSettings.applyDeadzone(DynamicInput.getAxisValue(gamepad1, GamepadSettings.gamepad1ButtonMapping.powerY)));
        powerRotate = GamepadSettings.applyBoostCurve(GamepadSettings.applyDeadzone(DynamicInput.getAxisValue(gamepad1, GamepadSettings.gamepad1ButtonMapping.powerRotate)));
        toggleDeliveryClaw.update(DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.toggleDeliveryClaw)
                || DynamicInput.getButtonState(gamepad1, GamepadSettings.gamepad1ButtonMapping.toggleDeliveryClaw));
        subWithdraw = DynamicInput.getButtonState(gamepad1, GamepadSettings.gamepad1ButtonMapping.subWithdraw);

        specIntake = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.specIntake)
                || DynamicInput.getButtonState(gamepad1, GamepadSettings.gamepad1ButtonMapping.specIntake)
                || DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.sideDeposit);

        ascendToggle.update(DynamicInput.getButtonState(gamepad1, GamepadSettings.gamepad1ButtonMapping.ascendToRobotHeaven));
        ascendToRobotHeaven = ascendToggle.isPressed();

        // Gamepad 2
        extension = GamepadSettings.applyBoostCurve(GamepadSettings.applyDeadzone(DynamicInput.getAxisValue(gamepad2, GamepadSettings.gamepad2ButtonMapping.extension)));
        sideDeposit = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.sideDeposit);
        centerYaw = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.centerYaw);
        yaw = GamepadSettings.applyBoostCurve(GamepadSettings.applyDeadzone(DynamicInput.getAxisValue(gamepad2, GamepadSettings.gamepad2ButtonMapping.yaw)));
        diffyX = DynamicInput.getAxisValue(gamepad2, GamepadSettings.gamepad2ButtonMapping.diffyX);
        diffyY = DynamicInput.getAxisValue(gamepad2, GamepadSettings.gamepad2ButtonMapping.diffyY);
        prepareIntake = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.prepareIntake);
        pitchHigh = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.pitchHigh);
        sampleTransferButton.pressed = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.sampleTransfer);
        intakeButton.pressed = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.intake);
        release = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.release);
        specDeliveryPosition = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.specDelivery);
        liftUp = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.liftUp);
        liftDown = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.liftDown);
        swing.pressed = DynamicInput.getButtonState(gamepad2, GamepadSettings.gamepad2ButtonMapping.swing);
    }
}
