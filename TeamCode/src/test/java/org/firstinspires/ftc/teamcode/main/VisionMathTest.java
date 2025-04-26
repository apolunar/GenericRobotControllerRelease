package org.firstinspires.ftc.teamcode.main;

import static org.firstinspires.ftc.teamcode.robotcore.command.extension.AlignSampleLimelight.computeSlideExtension;
import static org.junit.Assert.assertEquals;

import com.qualcomm.robotcore.util.RobotLog;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.RobolectricTestRunner;

@RunWith(RobolectricTestRunner.class)
public class VisionMathTest {
    @Test
    public void testComputeSlideExtension() {
        double fx = 300.0;
        double fy = 300.0;
        double cx = 160.0;
        double cy = 120.0;

        // Camera pose
        double cameraHeightMeters = 1.0; // 1 meter high
        double cameraPitchRadians = Math.toRadians(30); // 30 degrees downward

        // Target appears at the center of the image
        double xPixel = cx;
        double yPixel = cy;

        // Run the function
        double result = computeSlideExtension(
                xPixel, yPixel,
                fx, fy, cx, cy,
                cameraHeightMeters,
                cameraPitchRadians
        );

        // Expected: if the target is in the center and camera is pitched down,
        // the ray intersects the ground at a known distance
        // For 30° pitch and 1 meter height, the horizontal distance should be:
        double expectedDistance = cameraHeightMeters / Math.tan(cameraPitchRadians);

        RobotLog.dd(this.getClass().getSimpleName(), "Expected distance: %s", expectedDistance);

        // Assert it's close (within a cm or two)
        assertEquals(expectedDistance, result, 0.01);
    }

}
