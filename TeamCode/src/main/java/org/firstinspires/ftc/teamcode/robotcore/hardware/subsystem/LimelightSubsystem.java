package org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.List;

import lombok.Getter;

/**
 * Limelight returns Tx and Ty values, which return angles for where a detected object is,
 * and trig is required to get pixel or distance values.
 * IMPORTANT: Tx and Ty are zero when no desired object is detected.
 */
public class LimelightSubsystem extends SubsystemBase {
    public Limelight3A limelight;

    public enum LimelightPipeline {
        DETECTOR,
        YELLOW,
        RED,
        BLUE,
    }

    @Getter
    private LimelightPipeline currentPipeline = LimelightPipeline.DETECTOR;
    @Getter
    private boolean limelightStreaming = false;
    @Getter
    private List<LLResultTypes.DetectorResult> detectorResults;

    public LimelightSubsystem(Limelight3A limelight) {
        this.limelight = limelight;
    }

    public LimelightSubsystem(HardwareMap hMap) {
        this(hMap.get(Limelight3A.class, "limelight"));
    }

    @Override
    public void periodic() {
        if (limelightStreaming) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                switch (currentPipeline) {
                    case DETECTOR:
                        // Access detector results
                        detectorResults = result.getDetectorResults();
                        for (LLResultTypes.DetectorResult dr : detectorResults) {
                            RobotLog.dd("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                        }
                        break;
                    default:
                        break;
                }

            }
        }
    }

    /**
     * Initializes limelight with polling rate of 100 Hz.
     * Separate method in case performance hit is not desired.
     */
    public void start() {
        limelight.pipelineSwitch(currentPipeline.ordinal() + 1);
        limelight.start();
        limelight.setPollRateHz(100);
        limelightStreaming = true;
    }

    public void stop() {
        limelight.stop();
        limelightStreaming = false;
    }

    /**
     * Switches the current pipeline to a new pipeline.
     * @param newPipeline The new pipeline to switch to:
     *                    TODO
     */
    public void setCurrentPipeline(LimelightPipeline newPipeline) {
        currentPipeline = newPipeline;
        limelight.pipelineSwitch(currentPipeline.ordinal() + 1);
    }
}
