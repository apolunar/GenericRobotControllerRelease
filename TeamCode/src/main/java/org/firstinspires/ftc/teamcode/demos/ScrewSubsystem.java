package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotcore.hardware.RobotSubsystem;

import lombok.Getter;

/**
 * Moves the delivery arm laterally up and down.
 */
@Config
public class ScrewSubsystem extends SubsystemBase implements RobotSubsystem {
    // CONFIG
    private static final String SCREW_NAME = "screw";
    private static final String SCREW_POTENTIOMETER_NAME = "pot";

    public static double MIN_SCREW_POS_VOLTAGE = 1.897;
    public static double MAX_SCREW_POS_VOLTAGE = 3.315;

    // HARDWARE
    @Getter private final DcMotorEx screwMotor;
    private final AnalogInput screwPotentiometer;

    public ScrewSubsystem(DcMotorEx screwMotor, AnalogInput screwPotentiometer) {
        this.screwMotor = screwMotor;
        this.screwPotentiometer = screwPotentiometer;
    }

    public ScrewSubsystem(HardwareMap hMap) {
        this(hMap.get(DcMotorEx.class, SCREW_NAME), hMap.get(AnalogInput.class, SCREW_POTENTIOMETER_NAME));
    }

    public void setOutput(double output) {
        if ( ( (screwPotentiometer.getVoltage() > MIN_SCREW_POS_VOLTAGE) || (output < 0) ) &&
                (screwPotentiometer.getVoltage() < MAX_SCREW_POS_VOLTAGE || (output > 0) ) ) {
            screwMotor.setPower(output);
        }
    }

    public double getPotentiometerVoltage() {
        return screwPotentiometer.getVoltage();
    }
}
