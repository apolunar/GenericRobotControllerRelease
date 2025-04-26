package org.firstinspires.ftc.teamcode.robotcore.tele;

public class Gamepad2ButtonMapping {
    // Arm controls
    public TeleConfig.GamepadAxis extension = TeleConfig.GamepadAxis.LEFT_STICK_Y;
    public TeleConfig.GamepadAxis yaw = TeleConfig.GamepadAxis.LEFT_STICK_X;
    public TeleConfig.GamepadAxis diffyX = TeleConfig.GamepadAxis.LEFT_STICK_Y;
    public TeleConfig.GamepadAxis diffyY = TeleConfig.GamepadAxis.RIGHT_STICK_Y;
    public TeleConfig.GamepadButton intake = TeleConfig.GamepadButton.RIGHT_TRIGGER;
    public TeleConfig.GamepadButton release = TeleConfig.GamepadButton.RIGHT_BUMPER;
    public TeleConfig.GamepadButton prepareIntake = TeleConfig.GamepadButton.A;
    public TeleConfig.GamepadButton pitchHigh = TeleConfig.GamepadButton.Y;
    public TeleConfig.GamepadButton sideDeposit = TeleConfig.GamepadButton.B;
    public TeleConfig.GamepadButton centerYaw = TeleConfig.GamepadButton.X;
    public TeleConfig.GamepadButton sampleTransfer = TeleConfig.GamepadButton.BACK;

    // Lift controls
    public TeleConfig.GamepadButton toggleDeliveryClaw = TeleConfig.GamepadButton.LEFT_TRIGGER;
    public TeleConfig.GamepadButton specDelivery = TeleConfig.GamepadButton.DPAD_UP;
    public TeleConfig.GamepadButton specIntake = TeleConfig.GamepadButton.DPAD_DOWN;
    public TeleConfig.GamepadButton liftUp = TeleConfig.GamepadButton.DPAD_RIGHT;
    public TeleConfig.GamepadButton liftDown = TeleConfig.GamepadButton.DPAD_LEFT;
    public TeleConfig.GamepadButton swing = TeleConfig.GamepadButton.LEFT_BUMPER;
}
