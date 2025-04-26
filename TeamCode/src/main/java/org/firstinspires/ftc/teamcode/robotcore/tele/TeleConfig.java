package org.firstinspires.ftc.teamcode.robotcore.tele;

public class TeleConfig {

    public enum DriverProfile {
        DEFAULT_PROFILE("Default profile", new GamepadSettings()),
        ISA_PROFILE("Isa's profile", new GamepadSettings() {
            {
                gamepad2ButtonMapping.liftUp = GamepadButton.DPAD_UP;
                gamepad2ButtonMapping.liftDown = GamepadButton.DPAD_DOWN;
                gamepad2ButtonMapping.specDelivery = GamepadButton.DPAD_RIGHT;
                gamepad2ButtonMapping.specIntake = GamepadButton.DPAD_LEFT;
            }
        }),
        AYANA_PROFILE("Ayana's profile", new GamepadSettings() {
            {

            }
        }),
        ANTONIN_PROFILE("Antonin's profile", new GamepadSettings() {
            {

            }
        }),
        JAMES_PROFILE("James's profile", new GamepadSettings() {
            {

            }
        });
        public final String name;
        public final GamepadSettings gamepadSettings;
        DriverProfile(String name, GamepadSettings gamepadSettings) {
            this.name = name;
            this.gamepadSettings = gamepadSettings;
        }
    };

    public enum GamepadButton {
        // Face buttons
        A, B, X, Y,

        // D-pad
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,

        // Shoulder buttons
        LEFT_BUMPER, RIGHT_BUMPER,

        // Center buttons
        START, BACK, GUIDE,

        // Stick buttons
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON,
        OPTIONS,
        RIGHT_TRIGGER, LEFT_TRIGGER
    }

    public enum GamepadAxis {
        LEFT_TRIGGER, RIGHT_TRIGGER,
        LEFT_STICK_X, LEFT_STICK_Y,
        RIGHT_STICK_X, RIGHT_STICK_Y
    }
}
