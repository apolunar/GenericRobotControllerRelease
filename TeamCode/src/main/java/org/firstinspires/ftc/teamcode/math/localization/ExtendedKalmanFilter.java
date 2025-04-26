package org.firstinspires.ftc.teamcode.math.localization;

/**
 * An Extended Kalman Filter implementation.
 * <p>
 * This class implements a generic EKF where the non­linear system and measurement models,
 * along with their Jacobians, are provided via functional interfaces.
 */
public class ExtendedKalmanFilter implements KalmanFilter {

    // Current state vector and covariance matrix.
    private double[] state;
    private double[][] covariance;

    // Nonlinear system and measurement functions with their Jacobians.
    private final StateTransitionFunction f;
    private final JacobianFunction F; // Jacobian of the state transition function.
    private final MeasurementFunction h;
    private final MeasurementJacobianFunction H; // Jacobian of the measurement function.

    // Process noise covariance matrix.
    private final double[][] processNoiseCovariance;

    /**
     * Constructs an ExtendedKalmanFilter.
     *
     * @param initialState            Initial state vector.
     * @param initialCovariance       Initial covariance matrix.
     * @param f                       Nonlinear state transition function.
     * @param F                       Jacobian of the state transition function.
     * @param h                       Nonlinear measurement function.
     * @param H                       Jacobian of the measurement function.
     * @param processNoiseCovariance  Process noise covariance matrix.
     */
    public ExtendedKalmanFilter(
            double[] initialState,
            double[][] initialCovariance,
            StateTransitionFunction f,
            JacobianFunction F,
            MeasurementFunction h,
            MeasurementJacobianFunction H,
            double[][] processNoiseCovariance) {
        this.state = initialState;
        this.covariance = initialCovariance;
        this.f = f;
        this.F = F;
        this.h = h;
        this.H = H;
        this.processNoiseCovariance = processNoiseCovariance;
    }

    /**
     * Predicts the state ahead by a time increment dt using the system model.
     * The covariance is updated as: P = Fk * P * Fk^T + Q.
     *
     * @param dt           Time increment in seconds.
     * @param controlInput Control input vector (may be null if not used).
     */
    @Override
    public void predict(double dt, double[] controlInput) {
        // Predict the new state using the nonlinear state transition function.
        state = f.apply(state, controlInput, dt);

        // Compute the Jacobian of the state transition function.
        // (Often computed at the previous state; here we assume evaluation at the new state.)
        double[][] Fk = F.compute(state, controlInput, dt);

        // Update the covariance: P = Fk * P * Fk^T + Q.
        covariance = matrixAdd(
                matrixMultiply(matrixMultiply(Fk, covariance), transpose(Fk)),
                processNoiseCovariance);
    }

    /**
     * Updates the state estimate using the measurement.
     * It computes the innovation, the Kalman gain, then updates the state and covariance.
     *
     * @param measurement           The measurement vector.
     * @param measurementCovariance The measurement noise covariance matrix.
     */
    @Override
    public void update(double[] measurement, double[][] measurementCovariance) {
        // Predict the measurement from the current state.
        double[] predictedMeasurement = h.apply(state);

        // Compute innovation (residual): y = measurement - predictedMeasurement.
        double[] innovation = vectorSubtract(measurement, predictedMeasurement);

        // Compute the measurement Jacobian.
        double[][] Hk = H.compute(state);

        // Innovation covariance: S = Hk * P * Hk^T + R.
        double[][] S = matrixAdd(
                matrixMultiply(matrixMultiply(Hk, covariance), transpose(Hk)),
                measurementCovariance);

        // Compute Kalman gain: K = P * Hk^T * inverse(S).
        double[][] K = matrixMultiply(
                matrixMultiply(covariance, transpose(Hk)),
                inverse(S));

        // Update state: x = x + K * y.
        state = vectorAdd(state, matrixVectorMultiply(K, innovation));

        // Update covariance: P = (I - K * Hk) * P.
        double[][] I = identityMatrix(state.length);
        covariance = matrixMultiply(
                matrixSubtract(I, matrixMultiply(K, Hk)),
                covariance);
    }

    /**
     * Returns the current state estimate.
     *
     * @return Current state vector.
     */
    @Override
    public double[] getState() {
        return state;
    }

    /**
     * Returns the current state covariance matrix.
     *
     * @return Current covariance matrix.
     */
    @Override
    public double[][] getCovariance() {
        return covariance;
    }

    /**
     * Resets the filter with a new state and covariance.
     *
     * @param initialState      New initial state vector.
     * @param initialCovariance New initial covariance matrix.
     */
    @Override
    public void reset(double[] initialState, double[][] initialCovariance) {
        state = initialState;
        covariance = initialCovariance;
    }

    // ---------- Matrix Helper Methods ----------

    private double[][] matrixMultiply(double[][] A, double[][] B) {
        int m = A.length;
        int n = B[0].length;
        int p = B.length;
        double[][] result = new double[m][n];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                for (int k = 0; k < p; k++) {
                    result[i][j] += A[i][k] * B[k][j];
                }
            }
        }
        return result;
    }

    private double[] matrixVectorMultiply(double[][] A, double[] x) {
        int m = A.length;
        int n = x.length;
        double[] result = new double[m];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                result[i] += A[i][j] * x[j];
            }
        }
        return result;
    }

    private double[][] transpose(double[][] A) {
        int m = A.length;
        int n = A[0].length;
        double[][] result = new double[n][m];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                result[j][i] = A[i][j];
            }
        }
        return result;
    }

    private double[][] matrixAdd(double[][] A, double[][] B) {
        int m = A.length;
        int n = A[0].length;
        double[][] result = new double[m][n];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                result[i][j] = A[i][j] + B[i][j];
            }
        }
        return result;
    }

    private double[][] matrixSubtract(double[][] A, double[][] B) {
        int m = A.length;
        int n = A[0].length;
        double[][] result = new double[m][n];
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                result[i][j] = A[i][j] - B[i][j];
            }
        }
        return result;
    }

    private double[] vectorSubtract(double[] a, double[] b) {
        int n = a.length;
        double[] result = new double[n];
        for (int i = 0; i < n; i++) {
            result[i] = a[i] - b[i];
        }
        return result;
    }

    private double[] vectorAdd(double[] a, double[] b) {
        int n = a.length;
        double[] result = new double[n];
        for (int i = 0; i < n; i++) {
            result[i] = a[i] + b[i];
        }
        return result;
    }

    private double[][] identityMatrix(int size) {
        double[][] I = new double[size][size];
        for (int i = 0; i < size; i++) {
            I[i][i] = 1.0;
        }
        return I;
    }

    /**
     * Computes the inverse of a square matrix using Gauss–Jordan elimination.
     * This implementation is best suited for small matrices.
     *
     * @param A Square matrix.
     * @return Inverse of A.
     */
    private double[][] inverse(double[][] A) {
        int n = A.length;
        double[][] augmented = new double[n][2 * n];

        // Create the augmented matrix [A | I].
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                augmented[i][j] = A[i][j];
            }
            augmented[i][i + n] = 1.0;
        }

        // Perform Gauss–Jordan elimination.
        for (int i = 0; i < n; i++) {
            double pivot = augmented[i][i];
            if (Math.abs(pivot) < 1e-10) {
                throw new RuntimeException("Matrix is singular or nearly singular");
            }
            // Scale the pivot row.
            for (int j = 0; j < 2 * n; j++) {
                augmented[i][j] /= pivot;
            }
            // Eliminate the pivot column entries in other rows.
            for (int k = 0; k < n; k++) {
                if (k != i) {
                    double factor = augmented[k][i];
                    for (int j = 0; j < 2 * n; j++) {
                        augmented[k][j] -= factor * augmented[i][j];
                    }
                }
            }
        }

        // Extract the inverse matrix.
        double[][] inv = new double[n][n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(augmented[i], n, inv[i], 0, n);
        }
        return inv;
    }

    // ---------- Functional Interfaces for Model Functions ----------

    /**
     * Functional interface for the nonlinear state transition function.
     * It computes the predicted state given the current state, control input, and time step.
     */
    public interface StateTransitionFunction {
        double[] apply(double[] state, double[] controlInput, double dt);
    }

    /**
     * Functional interface for computing the Jacobian of the state transition function.
     */
    public interface JacobianFunction {
        double[][] compute(double[] state, double[] controlInput, double dt);
    }

    /**
     * Functional interface for the nonlinear measurement function.
     * It computes the expected measurement for a given state.
     */
    public interface MeasurementFunction {
        double[] apply(double[] state);
    }

    /**
     * Functional interface for computing the Jacobian of the measurement function.
     */
    public interface MeasurementJacobianFunction {
        double[][] compute(double[] state);
    }
}
