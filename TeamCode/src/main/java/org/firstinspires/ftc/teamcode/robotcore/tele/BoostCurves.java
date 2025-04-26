package org.firstinspires.ftc.teamcode.robotcore.tele;

// Add this new class after the GamepadAxis enum
public class BoostCurves {
    public static double linear(double input) {
        return Math.max(0, Math.min(1, input));
    }

    // Quadratic - slower start, faster end
    public static double quadratic(double input) {
        input = Math.max(0, Math.min(1, input));
        return Math.pow(input, 2);
    }

    // Cubic - even slower start
    public static double cubic(double input) {
        input = Math.max(0, Math.min(1, input));
        return Math.pow(input, 3);
    }

    // Square root - faster start, slower end
    public static double squareRoot(double input) {
        input = Math.max(0, Math.min(1, input));
        return Math.sqrt(input);
    }

    // Sine wave - smooth S-curve
    public static double smooth(double input) {
        input = Math.max(0, Math.min(1, input));
        return (Math.sin((input - 0.5) * Math.PI) + 1) / 2;
    }

    // Step function - binary on/off at threshold
    public static double step(double input, double threshold) {
        return input >= threshold ? 1.0 : 0.0;
    }

    // Exponential - very slow start, very fast end
    public static double exponential(double input) {
        input = Math.max(0, Math.min(1, input));
        return (Math.exp(input * 3) - 1) / (Math.exp(3) - 1);
    }

    // Custom curve generator - allows for fine-tuning
    public static double custom(double input, double power) {
        input = Math.max(0, Math.min(1, input));
        return Math.pow(input, power);
    }
}
