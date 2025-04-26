package org.firstinspires.ftc.teamcode.robotcore.tele;

import com.qualcomm.robotcore.hardware.Gamepad;

import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class DynamicInput {
    public static boolean getButtonState(Gamepad gamepad, TeleConfig.GamepadButton button) {
        switch (button) {
            case A:
                return gamepad.a;
            case B:
                return gamepad.b;
            case X:
                return gamepad.x;
            case Y:
                return gamepad.y;
            case DPAD_UP:
                return gamepad.dpad_up;
            case DPAD_DOWN:
                return gamepad.dpad_down;
            case DPAD_LEFT:
                return gamepad.dpad_left;
            case DPAD_RIGHT:
                return gamepad.dpad_right;
            case LEFT_BUMPER:
                return gamepad.left_bumper;
            case RIGHT_BUMPER:
                return gamepad.right_bumper;
            case START:
                return gamepad.start;
            case BACK:
                return gamepad.back;
            case LEFT_STICK_BUTTON:
                return gamepad.left_stick_button;
            case RIGHT_STICK_BUTTON:
                return gamepad.right_stick_button;
            case GUIDE:
                return gamepad.guide;
            case OPTIONS:
                return gamepad.options;
            case LEFT_TRIGGER:
                return gamepad.left_trigger > GamepadSettings.trigger_threshold;
            case RIGHT_TRIGGER:
                return gamepad.right_trigger > GamepadSettings.trigger_threshold;
            default:
                throw new IllegalArgumentException("Unexpected button: " + button);
        }
    }

    public static double getAxisValue(Gamepad gamepad, TeleConfig.GamepadAxis axis) {
        switch (axis) {
            case LEFT_TRIGGER:
                return gamepad.left_trigger;
            case RIGHT_TRIGGER:
                return gamepad.right_trigger;
            case LEFT_STICK_X:
                return gamepad.left_stick_x;
            case LEFT_STICK_Y:
                return gamepad.left_stick_y;
            case RIGHT_STICK_X:
                return gamepad.right_stick_x;
            case RIGHT_STICK_Y:
                return gamepad.right_stick_y;
            default:
                throw new IllegalArgumentException("Unexpected axis: " + axis);
        }
    }
}
