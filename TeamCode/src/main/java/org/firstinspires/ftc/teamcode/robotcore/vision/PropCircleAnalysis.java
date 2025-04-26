package org.firstinspires.ftc.teamcode.robotcore.vision;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robotcore.game.Alliance;
import org.firstinspires.ftc.teamcode.robotcore.game.element.Prop;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

import lombok.Getter;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class PropCircleAnalysis extends OpenCvPipeline {
    private final Alliance alliance;
    private final double runtimeSeconds;

    public static Scalar lowerBlue = new Scalar(35.4, 85.0, 51.0);
    public static Scalar upperBlue = new Scalar(255.0, 119.0, 194.1);

    public static Scalar lowerRed = new Scalar(1.4, 165.8, 79.3);
    public static Scalar upperRed = new Scalar(157.3, 215.3, 255.0);

    @Getter private final ArrayList<Prop.PropPosition> latestPropPositions = new ArrayList<>();

    @Getter private Prop.PropPosition latestPropPosition;

    private final Mat ycrcbMat       = new Mat();
    private final Mat binaryMat      = new Mat();
    private final Mat maskedInputMat = new Mat();
    private final Mat gray           = new Mat();

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

        // Get mask of colors in proper range
        Core.inRange(ycrcbMat, alliance == Alliance.RED_ALLIANCE ? lowerRed : lowerBlue,
                alliance == Alliance.RED_ALLIANCE ? upperRed : upperBlue, binaryMat);
        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        maskedInputMat.release();

        // Make masked mat
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        ///

        Imgproc.cvtColor(maskedInputMat, gray, Imgproc.COLOR_BGR2GRAY);

        // Apply GaussianBlur to reduce noise and help circle detection
        Imgproc.GaussianBlur(gray, gray, new Size(9, 9), 2, 2);

        // Use HoughCircles to detect circles
        Mat circles = new Mat();
        Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.5, 20, 50, 40, 0, 50);

        int springWindow = input.cols() / 2;

        // Draw circles on the original image
        for (int i = 0; i < circles.cols(); i++) {
            double[] circle = circles.get(0, i);
            Point center = new Point(Math.round(circle[0]), Math.round(circle[1]));
            int radius = (int) Math.round(circle[2]);

            RobotLog.dd(this.getClass().getSimpleName(), "New circle : (radius: %s center: %s)", radius, center);

            // Draw the circle center
            Imgproc.circle(input, center, 3, new Scalar(0, 255, 0), -1, 8, 0);

            // Draw the circle outline
            Imgproc.circle(input, center, radius, new Scalar(0, 0, 255), 3, 8, 0);

            if (center.x > springWindow) {
                latestPropPositions.add(Prop.PropPosition.RIGHT);
            }
            else {
                latestPropPositions.add(Prop.PropPosition.CENTER);
            }
        }

        if (runtime.seconds() >= runtimeSeconds*0.99) {
            RobotLog.dd(this.getClass().getSimpleName(), "Latest prop position size : %s", latestPropPositions.size());

            if (latestPropPositions.size() < 60) {
                latestPropPosition = Prop.PropPosition.LEFT;
            } else {
                // Get mode of ArrayList
                latestPropPosition = propPositionMode();
            }

            // Reset so that latestPropPosition is the mode of the last second of readings
            reset();
        }

        return maskedInputMat;
    }

    public void reset() {
        runtime.reset();
        latestPropPositions.clear();
    }

    public Prop.PropPosition propPositionMode() {
        int n = latestPropPositions.size();

        int maxcount = 0;
        Prop.PropPosition elementHavingMaxFreq = Prop.PropPosition.NONE;
        for (int i = 0; i < n; i++) {
            int count = 0;
            for (int j = 0; j < n; j++) {
                if (latestPropPositions.get(i) == latestPropPositions.get(j)) {
                    count++;
                }
            }

            if (count > maxcount) {
                maxcount = count;
                elementHavingMaxFreq = latestPropPositions.get(i);
            }
        }

        return elementHavingMaxFreq;
    }
}
