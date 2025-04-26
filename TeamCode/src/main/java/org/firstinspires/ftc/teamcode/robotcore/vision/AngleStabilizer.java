package org.firstinspires.ftc.teamcode.robotcore.vision;

import java.util.*;

public class AngleStabilizer {
    private final int bufferSize;
    private final Deque<Double> angleBuffer;
    private final Map<Double, Integer> frequencyMap;

    public AngleStabilizer(int size) {
        this.bufferSize = size;
        this.angleBuffer = new ArrayDeque<>();
        this.frequencyMap = new HashMap<>();
    }

    public double addAndGetStableAngle(double angle) {
        // Round to the nearest 0.01 to avoid noise and floating point issues
        double rounded = Math.round(angle * 100.0) / 100.0;

        angleBuffer.addLast(rounded);
        frequencyMap.put(rounded, frequencyMap.getOrDefault(rounded, 0) + 1);

        if (angleBuffer.size() > bufferSize) {
            double removed = angleBuffer.removeFirst();
            frequencyMap.put(removed, frequencyMap.get(removed) - 1);
            if (frequencyMap.get(removed) == 0) {
                frequencyMap.remove(removed);
            }
        }

        return getMode();
    }

    private double getMode() {
        double mode = 0;
        int maxCount = -1;
        for (Map.Entry<Double, Integer> entry : frequencyMap.entrySet()) {
            if (entry.getValue() > maxCount) {
                maxCount = entry.getValue();
                mode = entry.getKey();
            }
        }
        return mode;
    }
}
