package org.firstinspires.ftc.teamcode.demos.concept;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

@TeleOp(name = "!")
@Disabled
public class ConceptPurePursuit extends CommandOpMode {
    @Override
    public void initialize() {
        ChassisSubsystem chassisSubsystem = new ChassisSubsystem(hardwareMap);

        Waypoint p1 = new StartWaypoint(0, 0);
        Waypoint p2 = new EndWaypoint(100, 0, 0, 1, 1, 1, 1, 1);

        telemetry.log().add("Latest pose : %s", chassisSubsystem.getLatestPose());

        // TODO: Well this sucks, we can no longer use anything that depends on MecanumDrive because of our custom class!
//        PurePursuitCommand ppCommand = new PurePursuitCommand(
//                chassisSubsystem.getDrive(), chassisSubsystem.getOdometrySubsystem(),
//                p1, p2
//        );

//        schedule(ppCommand);
        register(chassisSubsystem);
    }
}