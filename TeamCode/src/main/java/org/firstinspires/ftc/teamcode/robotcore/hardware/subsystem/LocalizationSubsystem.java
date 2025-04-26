package org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.math.localization.ExtendedKalmanFilter;

// Assuming you have a Pose2d class representing (x, y, theta)
public class LocalizationSubsystem {

    private HolonomicOdometry holonomicOdometry;
    private OdometrySubsystem odometrySubsystem;
    private ExtendedKalmanFilter kalmanFilter;
    private IMU imu;
    private double lastTimestamp;

    /**
     * Initializes the localization system with the odometry, IMU, and starting pose.
     */
    public LocalizationSubsystem(HolonomicOdometry odometry, IMU imu, Pose2d startingPose) {
        this.holonomicOdometry = odometry;
        if (startingPose != null) {
            holonomicOdometry.updatePose(startingPose);
        }
        odometrySubsystem = new OdometrySubsystem(holonomicOdometry);
        this.imu = imu;

        // Use starting pose for initial EKF state: [x, y, theta]
        double[] initialState = new double[] {
                startingPose.getX(),
                startingPose.getY(),
                startingPose.getHeading()
        };

        // Initial uncertainty (covariance)
        double[][] initialCovariance = new double[][] {
                {1,   0,   0},
                {0,   1,   0},
                {0,   0, 0.1}
        };

        // Create the ExtendedKalmanFilter
        kalmanFilter = new ExtendedKalmanFilter(
                initialState,
                initialCovariance,
                // State transition function: updates pose based on control input and dt
                (state, controlInput, dt) -> {
                    double x = state[0];
                    double y = state[1];
                    double theta = state[2];

                    // Control input: { forwardSpeed, strafeSpeed, turnSpeed }
                    double forward = controlInput[0];
                    double strafe  = controlInput[1];
                    double turn    = controlInput[2];

                    // Update state using a simple kinematic model
                    double newX = x + (forward * Math.cos(theta) - strafe * Math.sin(theta)) * dt;
                    double newY = y + (forward * Math.sin(theta) + strafe * Math.cos(theta)) * dt;
                    double newTheta = theta + turn * dt;
                    return new double[] { newX, newY, newTheta };
                },
                // Jacobian of the state transition function
                (state, controlInput, dt) -> {
                    double theta = state[2];
                    double forward = controlInput[0];
                    double strafe  = controlInput[1];

                    double[][] F = new double[3][3];
                    // Partial derivatives for x update
                    F[0][0] = 1;
                    F[0][1] = 0;
                    F[0][2] = - (forward * Math.sin(theta) + strafe * Math.cos(theta)) * dt;
                    // Partial derivatives for y update
                    F[1][0] = 0;
                    F[1][1] = 1;
                    F[1][2] = (forward * Math.cos(theta) - strafe * Math.sin(theta)) * dt;
                    // Theta update
                    F[2][0] = 0;
                    F[2][1] = 0;
                    F[2][2] = 1;
                    return F;
                },
                // Measurement function: we assume the measurement vector is directly the pose [x, y, theta]
                (state) -> state,
                // Measurement Jacobian: since measurement is direct, it’s the identity matrix
                (state) -> new double[][] {
                        {1, 0, 0},
                        {0, 1, 0},
                        {0, 0, 1}
                },
                // Process noise covariance matrix
                new double[][] {
                        {0.1, 0,   0},
                        {0,   0.1, 0},
                        {0,   0, 0.05}
                }
        );

        lastTimestamp = System.currentTimeMillis() / 1000.0;
    }

    /**
     * Call this method periodically to update the localization state.
     *
     * @param forwardSpeed Commanded forward speed.
     * @param strafeSpeed  Commanded strafe (lateral) speed.
     * @param turnSpeed    Commanded turning speed.
     */
    public void updateLocalization(double forwardSpeed, double strafeSpeed, double turnSpeed) {
        double currentTimestamp = System.currentTimeMillis() / 1000.0;
        double dt = currentTimestamp - lastTimestamp;
        lastTimestamp = currentTimestamp;

        // Use the drive command as control input for the prediction step.
        double[] controlInput = new double[] { forwardSpeed, strafeSpeed, turnSpeed };
        kalmanFilter.predict(dt, controlInput);

        // Gather measurements: odometry provides (x,y) and the IMU gives the heading (theta).
        Pose2d odometryPose = holonomicOdometry.getPose();
        double measuredX = odometryPose.getX();
        double measuredY = odometryPose.getY();
        double measuredTheta = imu.getRobotYawPitchRollAngles().getPitch();
        double[] measurement = new double[] { measuredX, measuredY, measuredTheta };

        // Define measurement noise covariance; these values need to be tuned for your sensors.
        double[][] measurementCovariance = new double[][] {
                {0.2, 0,   0},
                {0,   0.2, 0},
                {0,   0, 0.1}
        };

        // Update the EKF with the sensor measurements.
        kalmanFilter.update(measurement, measurementCovariance);
    }

    /**
     * Returns the current estimated pose from the EKF.
     */
    public Pose2d getEstimatedPose() {
        double[] state = kalmanFilter.getState();
        return new Pose2d(state[0], state[1], new Rotation2d(state[2]));
    }
}
