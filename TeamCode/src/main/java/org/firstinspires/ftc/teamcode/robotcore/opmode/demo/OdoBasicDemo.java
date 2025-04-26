package org.firstinspires.ftc.teamcode.robotcore.opmode.demo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp
@Disabled
public class OdoBasicDemo extends LinearOpMode {
    private static final String TAG = "Odo Basic Demo";

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFrontMotor  = hardwareMap.get(DcMotorEx.class, "leftFront");
        DcMotorEx rightFrontMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        DcMotorEx leftBackMotor   = hardwareMap.get(DcMotorEx.class, "leftBack");
        DcMotorEx rightBackMotor  = hardwareMap.get(DcMotorEx.class, "rightBack");

        rightFrontMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Forward is where the front-facing camera is pointing
//        leftOdometer   = leftBackMotor.encoder;
//        rightOdometer  = rightFrontMotor.encoder;
//        centerOdometer = rightBackMotor.encoder;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            RobotLog.dd(TAG, "larger   : %s", leftBackMotor.getVelocity() > rightFrontMotor.getVelocity()  ? "left" : "right");
            telemetry.addData(TAG, "larger   : %s", leftBackMotor.getVelocity() > rightFrontMotor.getVelocity()  ? "left" : "right");
            RobotLog.dd(TAG, "left     : %s", leftBackMotor.getVelocity());
            telemetry.addData(TAG, "left     : %s", leftBackMotor.getVelocity());
            RobotLog.dd(TAG, "right    : %s", rightFrontMotor.getVelocity());
            telemetry.addData(TAG, "right    : %s", rightFrontMotor.getVelocity());

            RobotLog.dd(TAG, "center    : %s", rightBackMotor.getVelocity());
            telemetry.addData(TAG, "center    : %s", rightBackMotor.getVelocity());

            RobotLog.dd(TAG, "left ticks     : %s", leftBackMotor.getCurrentPosition());
            telemetry.addData(TAG, "left ticks     : %s", leftBackMotor.getCurrentPosition());
            RobotLog.dd(TAG, "right ticks    : %s", rightFrontMotor.getCurrentPosition());
            telemetry.addData(TAG, "right ticks    : %s", rightFrontMotor.getCurrentPosition());

            RobotLog.dd(TAG, "center ticks    : %s", rightBackMotor.getCurrentPosition());
            telemetry.addData(TAG, "center ticks    : %s", rightBackMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}
