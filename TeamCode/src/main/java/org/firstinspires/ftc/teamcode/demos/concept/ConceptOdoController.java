package org.firstinspires.ftc.teamcode.demos.concept;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

@TeleOp(name = "!Drive By Odo")
@Disabled
public class ConceptOdoController extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);
        CommandScheduler.getInstance().registerSubsystem(chassisSubsystem);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            ChassisSpeeds speeds = new ChassisSpeeds(
                    gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x
            );

//            chassisSubsystem.driveChassisSpeeds(speeds);
        }
    }
}