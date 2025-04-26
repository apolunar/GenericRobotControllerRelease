package org.firstinspires.ftc.teamcode.gamepad;

public class MyGamepadButton {
    private boolean wasPressed = false;
    public boolean pressed = false;


    public boolean wasJustPressed() {
        boolean justPressed = pressed && !wasPressed;
        wasPressed = pressed;
        return justPressed;
    }

    public boolean wasJustPressed(boolean currentButtonState) {
        this.pressed = currentButtonState;
        return wasJustPressed();
    }
}
