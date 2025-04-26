package org.firstinspires.ftc.teamcode.robotcore.opmode.util.tuning;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.OdoWheelSpeeds;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

/**
 * Configured for lateral feedforward tuning.
 */
@TeleOp(group = "2")
public class FeedforwardTuner extends LinearOpMode {
    private final String TAG = "FeedforwardTuner";

    private final double MOTOR_POWER_INCREMENT = 0.1;
    private final double LOG_TIME = 1.0;
    private final int ACCELERATION_WAIT_MS = 200;

    @Override
    public void runOpMode() throws InterruptedException {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        ElapsedTime timer = new ElapsedTime();
        OdoWheelSpeeds odoWheelSpeeds;

        chassisSubsystem.getLeftOdometer().reset();
        chassisSubsystem.getRightOdometer().reset();
        chassisSubsystem.getCenterOdometer().reset();

        double currentPower = 0.1;

        waitForStart();

        while (currentPower <= 1.0 && !isStopRequested()) {
            telemetry.clear();

            // LATERAL
            chassisSubsystem.getDrive().driveWithMotorPowers(-currentPower,
                    currentPower,
                    currentPower,
                    -currentPower);
            // FORWARD
//            chassisSubsystem.getDrive().driveWithMotorPowers(-currentPower,
//                    -currentPower,
//                    -currentPower,
//                    -currentPower);
            sleep(ACCELERATION_WAIT_MS);
            timer.reset();

            while (timer.seconds() < LOG_TIME) {
                telemetry.addData("Power setting", currentPower);
                telemetry.addData("Time", timer.seconds());
                // NOTE: Removing any of these logged data points makes processing really annoying
                RobotLog.dd(TAG, "Power setting : %s", currentPower);
                RobotLog.dd(TAG, "Time : %s", timer.seconds());

//                odoWheelSpeeds = chassisSubsystem.getOdoWheelSpeeds();
//                RobotLog.dd(TAG, "Left odo vel  : %s", odoWheelSpeeds.leftMetersPerSecond);
//                RobotLog.dd(TAG, "Right odo vel : %s", odoWheelSpeeds.rightMetersPerSecond);
//                RobotLog.dd(TAG, "Center odo vel : %s", odoWheelSpeeds.centerMetersPerSecond);

                RobotLog.dd(TAG, "Left ticks   : %s", chassisSubsystem.getLeftOdometer().getPosition());
                RobotLog.dd(TAG, "Right ticks  : %s", chassisSubsystem.getRightOdometer().getPosition());
                RobotLog.dd(TAG, "Center ticks : %s", chassisSubsystem.getCenterOdometer().getPosition());


                chassisSubsystem.periodic();
                telemetry.update();
            }

            currentPower += MOTOR_POWER_INCREMENT;
            if (currentPower > 0.3) {
                // Some time for humans to reset the robot
                chassisSubsystem.getDrive().driveWithMotorPowers(0,0,0,0);
                telemetry.addLine("======== RESET ROBOT ========");
                telemetry.update();
                sleep(5000);
            }
        }

        RobotLog.dd(TAG, "========= TEST COMPLETE =========");
    }
}
