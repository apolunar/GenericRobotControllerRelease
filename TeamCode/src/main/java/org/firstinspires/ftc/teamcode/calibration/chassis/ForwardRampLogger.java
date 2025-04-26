package org.firstinspires.ftc.teamcode.calibration.chassis;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.robotcore.hardware.subsystem.ChassisSubsystem;

import java.util.List;
import java.util.Map;

/**
 * NOTE TO SELF:
 * It looks like the MyRobot class may have no use and instead all commands can be defined via CommandOpMode
 * However, MyRobot could migrate to RobotOpMode which automatically does some fancy things with a robot config
 * to run the robot in an easy way!
 */
@TeleOp(group = "Tuning")
@Disabled
public class ForwardRampLogger extends OpMode {
    ChassisSubsystem chassisSubsystem;

    double POWER_PER_SEC = 0.1;
    double POWER_MAX     = 0.9;

    List<LynxModule> hubs;
    private VoltageSensor myControlHubVoltageSensor;
    Tuning.MidpointTimer midpointTimer;

    Map<Double, Double> timePowerMap;
    Map<Double, Double> timeVoltageMap;

    @Override
    public void init() {
        chassisSubsystem = new ChassisSubsystem(hardwareMap);
        myControlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        chassisSubsystem.register();

        midpointTimer = new Tuning.MidpointTimer();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        telemetry.addLine("Running");

        CommandScheduler.getInstance().run();

        for (Motor motor : chassisSubsystem.getMotors()) {
            double powerAtRuntime = powerAtRuntime(time);
            motor.set(powerAtRuntime);

            timePowerMap.put(time, powerAtRuntime);
        }

        timeVoltageMap.put(time, myControlHubVoltageSensor.getVoltage());

        for (LynxModule module : hubs) {
            module.clearBulkCache();
        }
    }

    private double powerAtRuntime(double seconds) {
        return Math.min(POWER_PER_SEC * seconds, POWER_MAX);
    }
}