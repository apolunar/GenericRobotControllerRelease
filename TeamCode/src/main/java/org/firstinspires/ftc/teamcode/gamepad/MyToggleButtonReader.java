package org.firstinspires.ftc.teamcode.gamepad;

import lombok.Getter;

/**
 * Utility class to handle boolean logic for a toggle button.
 * Ensure to call the update function.
 */
public class MyToggleButtonReader {
    @Getter
    private boolean isPressed = false;
    private boolean lastButtonState = false;

    public void update(boolean currentState) {
        boolean justPressed = currentState && !lastButtonState;
        lastButtonState = currentState;
        if (justPressed) { // toggle the button if it was just pressed
            isPressed = !isPressed;
        }
    }
}
