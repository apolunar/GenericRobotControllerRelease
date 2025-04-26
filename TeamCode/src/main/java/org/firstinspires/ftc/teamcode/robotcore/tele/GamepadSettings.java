package org.firstinspires.ftc.teamcode.robotcore.tele;

import com.acmerobotics.dashboard.config.Config;

/**
 * Gamepad settings and button maps statically applied across all references (e.g., changing a profile will auto update everything)
 */
@Config
public class GamepadSettings {
    /**
     * Sensitivity multiplier for stick input
     */
    public static double trigger_threshold = 0.1;

    /**
     * Deadzone for stick inputs to prevent drift
     */
    public static double stick_deadzone = 0.05;

    /**
     * Button mappings, statically overridable
     */
    public static Gamepad1ButtonMapping gamepad1ButtonMapping = new Gamepad1ButtonMapping();
    public static Gamepad2ButtonMapping gamepad2ButtonMapping = new Gamepad2ButtonMapping();

    /**
     * Applies a mathematical curve to the boost input to adjust control response
     *
     * @param input Raw input value between 0 and 1
     * @return Modified input value between 0 and 1
     */
    public static double applyBoostCurve(double input) {
        // Default implementation: simple clamp between 0 and 1
        return Math.max(0, Math.min(1, input));
    }

    public static double applyDeadzone(double value) {
        return Math.abs(value) > stick_deadzone ? value : 0;
    }
}
