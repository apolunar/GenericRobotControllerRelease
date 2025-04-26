package org.firstinspires.ftc.teamcode.robotcore.vision;

import android.graphics.Canvas;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotConfig;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.CameraSubsystem;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.*;

import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import lombok.Getter;
import lombok.Setter;

public class SampleSampler implements VisionProcessor {
    private final Telemetry telemetry;

    public SampleSampler(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public static final Scalar BLUE = new Scalar(0, 0, 255);
    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final Scalar RED = new Scalar(255, 0, 0);

    // Instance fields declared once (e.g., in the constructor or as class members)
    private final List<Mat> channels = new ArrayList<>();
    private final Mat ycrcb = new Mat();
    private final Mat colorCorrected = new Mat();
    private final Mat edges = new Mat();
    private final Mat dilated = new Mat();
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    private final List<MatOfPoint> contours = new ArrayList<>();
    private final Mat hierarchy = new Mat();
    private final Mat binaryMat = new Mat();
    private final Mat maskedInputMat = new Mat();
    private final Mat labImage = new Mat();
    private final List<Mat> labChannels = new ArrayList<>(); // used for Lab splitting
//    private final Mat inputRotRects = new Mat();
    private final Mat inputContours = new Mat();


    // Predefined target brightness value
    double targetMean = -30;

    public static Scalar lowerRed = new Scalar(0, 0, 0);
    public static Scalar upperRed = new Scalar(255, 255, 120.4);

    public static Scalar lowerYellow = new Scalar(160., 89.3, 56.7);
    public static Scalar upperYellow = new Scalar(255, 157.3, 138.8);

    public static Scalar lowerBlue = new Scalar(0., 134.6, 100.0);
    public static Scalar upperBlue = new Scalar(255, 255, 255);

    public enum SampleVision {
        RED(lowerRed, upperRed),
        YELLOW(lowerYellow, upperYellow),
        BLUE(lowerBlue, upperBlue),;
        public Scalar lowerSample;
        public Scalar upperSample;
        SampleVision(Scalar lowerSample, Scalar upperSample) {
            this.lowerSample = lowerSample;
            this.upperSample = upperSample;
        }
    }

    @Setter
    @Getter
    private volatile SampleVision sampleVision = SampleVision.RED;
    SampleVision[] visions = SampleVision.values();

    public void nextSampleVision() {
        int currentStageNum = sampleVision.ordinal();

        int nextStageNum = currentStageNum + 1;

        if(nextStageNum >= visions.length)
        {
            nextStageNum = 0;
        }

        sampleVision = visions[nextStageNum];
    }

    private ArrayList<MatOfPoint> contoursByArea = new ArrayList<>();
    private MatOfPoint2f contoursByArea2f = new MatOfPoint2f();
    @Getter
    private volatile ArrayList<RotatedRect> contoursByAreaRotRects = new ArrayList<>();

    int minAreaContour = 2000;
    int maxAreaContour = 3000000;
    int minAreaRec = 30000;
    int maxAreaRec = 30000000;
    double targetRatio = 3.5/1.5;
    double ratioTolerance = 0.8;

    private Mat input = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    public Mat cropTop(Mat input, int cutPixels) {
        // Define the region of interest (ROI) that excludes the top part
        Rect roi = new Rect(0, cutPixels, input.cols(), input.rows() - cutPixels);

        // Return the cropped sub-matrix
        return new Mat(input, roi);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Very expensive
//        Calib3d.undistort(frame, input, CameraSubsystem.armCameraMatrix, CameraSubsystem.armDistCoeffs);
        frame = cropTop(frame, 160);
        frame.copyTo(input);

        // --- Step 1: Color Correction using YCrCb ---
        // Convert input from BGR to YCrCb.
        Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_BGR2YCrCb);

        // Split into channels (assume channels list has been preallocated and cleared)
        channels.clear();
        Core.split(ycrcb, channels);

        // Equalize histogram on the Y channel (channels.get(0))
        Imgproc.equalizeHist(channels.get(0), channels.get(0));

        // Compute the current mean brightness.
        Scalar meanScalar = Core.mean(channels.get(0));
        double currentMean = meanScalar.val[0];

        // Compute the brightness shift to achieve the target mean.
        double beta = targetMean - currentMean;
        channels.get(0).convertTo(channels.get(0), -1, 1, beta);

        // Merge the channels back and convert from YCrCb to BGR.
        Core.merge(channels, ycrcb);
        Imgproc.cvtColor(ycrcb, colorCorrected, Imgproc.COLOR_YCrCb2BGR);

        // --- Step 2: Convert to Lab for Improved Color Discrimination ---
        Imgproc.cvtColor(colorCorrected, labImage, Imgproc.COLOR_BGR2Lab);

        // Split Lab image into its channels.
        labChannels.clear();
        Core.split(labImage, labChannels);

        // Optionally, equalize the L channel to further reduce lighting variance.
        Imgproc.equalizeHist(labChannels.get(0), labChannels.get(0));

        // Merge channels back into a Lab image.
        Core.merge(labChannels, labImage);

        // --- Step 4: Apply the binary mask to the color-corrected image ---
//        Core.inRange(labImage, lowerBlue, upperBlue, binaryMat);
        Core.inRange(labImage, sampleVision.lowerSample, sampleVision.upperSample, binaryMat);
        maskedInputMat.release();
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        Imgproc.Canny(maskedInputMat, edges, 100, 200);
        Imgproc.dilate(edges, dilated, kernel);

        contours.clear();
        Imgproc.findContours(dilated, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        contoursByArea.clear();
        input.copyTo(inputContours);
        for(MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            // Note that the following debugging does not work but is interesting
//            Imgproc.putText(inputContours, "center", center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, BLUE, 2);
//            Imgproc.putText(inputContours, String.format("area: %s", area), new Point(cX - 50, cY - 50), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, BLUE, 2);
//            if (area > 100) {
//                telemetry.addData("Found contour with area: ", area);
//            }
            if((area >= minAreaContour) && (area <= maxAreaContour)) {
//                Moments moments = Imgproc.moments(contour);
//                double cX = moments.m10 / moments.m00;
//                double cY = moments.m01 / moments.m00;
//                Point center = new Point(cX - 20, cY - 20);
//                drawTagText(center, String.format("Contour: (area: %.2f)", area), frame);
                contoursByArea.add(contour);
            }
        }

        contoursByAreaRotRects.clear();
        for(MatOfPoint points : contoursByArea) {
            contoursByArea2f.release();
            points.convertTo(contoursByArea2f, CvType.CV_32F);

            RotatedRect rect = Imgproc.minAreaRect(contoursByArea2f);
            double width = rect.size.width;
            double height = rect.size.height;
            double area = width * height;
//            telemetry.addData("Found rect with area: ", area);

//            double aspectRatio = width > height ? width / height : height / width;
//            double tolerance = ratioTolerance * targetRatio;
            if ((area <= minAreaRec) || (area >= maxAreaRec)) {
                continue; // Discard rectangles that don't match the target aspect ratio or area
            }

            contoursByAreaRotRects.add(rect);
        }

//        input.copyTo(inputRotRects);
        // Draw rects
        for(RotatedRect rect : contoursByAreaRotRects) {
            if(rect != null) {
                double width = rect.size.width;
                double height = rect.size.height;
                double area = width * height;

//                drawTagText(rect, String.format("Rect: (width: %.2f, height: %.2f, area: %.2f)", width, height, area), frame);

                Point[] rectPoints = new Point[4];
                rect.points(rectPoints);
                MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

                Imgproc.polylines(frame, Collections.singletonList(matOfPoint), true, BLUE, 3);
            }
        }

        if (RobotConfig.DEBUG) {
            Imgproc.drawContours(frame, contoursByArea, -1, RED, 3);
        }

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }


    static void drawTagText(RotatedRect rect, String text, Mat mat)
    {
        drawTagText(new Point(rect.center.x, rect.center.y), text, mat);
    }

    static void drawTagText(Point center, String text, Mat mat)
    {
        Imgproc.putText(
                mat, // The buffer we're drawing on
                text, // The text we're drawing
                new Point( // The anchor point for the text
                        center.x-50,  // x anchor point
                        center.y+25), // y anchor point
                Imgproc.FONT_HERSHEY_PLAIN, // Font
                1, // Font size
                GREEN, // Font color
                1); // Font thickness
    }

    public RotatedRect getLargestRect() {
        if (contoursByAreaRotRects == null || contoursByAreaRotRects.isEmpty()) {
            return null;
        }

        RotatedRect largestRect = null;
        double maxArea = 0;

        for (RotatedRect rect : contoursByAreaRotRects) {
            // Calculate area using width and height.
            double area = rect.size.width * rect.size.height;
            if (area > maxArea) {
                maxArea = area;
                largestRect = rect;
            }
        }
        return largestRect;
    }
}
