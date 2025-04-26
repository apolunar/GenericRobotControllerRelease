package org.firstinspires.ftc.teamcode.calibration.chassis;

public class Tuning {
    public static class MidpointTimer {
        private final double beginTs = System.nanoTime();
        private double lastTime = 0;

        public double seconds() {
            return 1e-9 * (System.nanoTime() - beginTs);
        }

        public double addSplit() {
            double time = System.nanoTime() - beginTs;
            double midTimeSecs = 0.5e-9 * (lastTime + time);
            lastTime = time;
            return midTimeSecs;
        }
    }
}
