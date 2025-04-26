package org.firstinspires.ftc.teamcode.math.localization;

/**
 * Interface for a generic Kalman Filter.
 * <p>
 * This filter provides methods for state prediction and measurement update.
 * Implementations should define how the state transition and measurement models work.
 */
public interface KalmanFilter {

    /**
     * Predicts the state ahead by a time increment.
     *
     * @param dt           Time increment in seconds.
     * @param controlInput Control input vector applied to the system.
     *                     This can be null if there is no control input.
     */
    void predict(double dt, double[] controlInput);

    /**
     * Updates the state estimate with a new measurement.
     *
     * @param measurement           Measurement vector.
     * @param measurementCovariance Covariance matrix representing measurement noise.
     */
    void update(double[] measurement, double[][] measurementCovariance);

    /**
     * Returns the current state estimate.
     *
     * @return Current state vector.
     */
    double[] getState();

    /**
     * Returns the current state covariance matrix.
     *
     * @return Current covariance matrix.
     */
    double[][] getCovariance();

    /**
     * Resets the filter with an initial state and covariance.
     *
     * @param initialState       Initial state vector.
     * @param initialCovariance  Initial covariance matrix.
     */
    void reset(double[] initialState, double[][] initialCovariance);
}
