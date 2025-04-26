package org.firstinspires.ftc.teamcode.math.controller;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.teamcode.logging.LogWrapper;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RiccatiEquationSolver;
import org.hipparchus.linear.RiccatiEquationSolverImpl;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import lombok.RequiredArgsConstructor;

public class LQRController {
    private RealMatrix K;
    private Future<RealMatrix> futureK;
    private final ExecutorService executor;

    public LQRController(double[][] AMat, double[][] BMat, double[][] QMat, double[][] RMat) {
        RealMatrix A = MatrixUtils.createRealMatrix(AMat);
        RealMatrix B = MatrixUtils.createRealMatrix(BMat);
        RealMatrix Q = MatrixUtils.createRealMatrix(QMat);
        RealMatrix R = MatrixUtils.createRealMatrix(RMat);
        RiccatiEquationSolver solver = new RiccatiEquationSolverImpl(A, B, Q, R);
        K = solver.getK();

        executor = Executors.newSingleThreadExecutor();
    }

    public RealMatrix getK() {
        if (futureK != null && futureK.isDone()) {
            try {
                K = futureK.get();
                futureK = null;
            } catch (InterruptedException | ExecutionException e) {
                e.printStackTrace();
            }
        }
        return K;
    }

    public double[] calculate(RealMatrix currentState, RealMatrix targetState) {
        if (K == null) return new double[]{0, 0};

        RealMatrix error = targetState.subtract(currentState);

        // Control output (u = -K * error)
        RealMatrix controlOutput = K.multiply(error).scalarMultiply(-1);
        LogWrapper.log(this.getClass().getSimpleName(), "Control output : %s", controlOutput);
        double power = controlOutput.getEntry(0, 0) / 100;

        return new double[]{power, power};
    }
}
