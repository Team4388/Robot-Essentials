package frc4388.utility.status;

import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkMax;

import frc4388.utility.status.Status.ReportLevel;

public class FaultSparkMax implements Queryable {
    private String name;
    private SparkMax motor;

    public static void addDevice(SparkMax motor, String name) {
        FaultSparkMax p = new FaultSparkMax();

        p.name = name;
        p.motor = motor;

        FaultReporter.register(p);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Status diagnosticStatus() {
        Status s = new Status();

        s.addReport(ReportLevel.INFO, "Firmware Version: " + motor.getFirmwareString());
        s.addReport(ReportLevel.INFO, "Voltage: " + motor.getBusVoltage());
        s.addReport(ReportLevel.INFO, "Current: " + motor.getOutputCurrent());
        s.addReport(ReportLevel.INFO, "Device temp (C): " + motor.getMotorTemperature());
        s.addReport(ReportLevel.INFO, "Position: " + motor.getEncoder().getPosition());
        s.addReport(ReportLevel.INFO, "Velocity: " + motor.getEncoder().getVelocity());
        s.addReport(ReportLevel.INFO, "Forward hard limit: " + motor.getForwardLimitSwitch().isPressed());
        s.addReport(ReportLevel.INFO, "Reverse hard limit: " + motor.getReverseLimitSwitch().isPressed());
        s.addReport(ReportLevel.INFO, "Forward soft limit: " + motor.getForwardSoftLimit().isReached());
        s.addReport(ReportLevel.INFO, "Reverse soft limit: " + motor.getReverseSoftLimit().isReached());

        // faults
        Faults faults = motor.getFaults();
        if(faults.can) {
            s.addReport(ReportLevel.ERROR, "CAN Fault (Joe Johnson)");
        }
        if(faults.escEeprom) {
            s.addReport(ReportLevel.ERROR, "Escape Eeprom. Cannot write to internal memory (oh god I don't want to think about what this means)");
        }
        if(faults.firmware) {
            s.addReport(ReportLevel.ERROR, "Firmware Fault");
        }
        if(faults.gateDriver) {
            s.addReport(ReportLevel.ERROR, "Gate Driver Fault");
        }
        if(faults.motorType) {
            s.addReport(ReportLevel.ERROR, "Motor type Fault");
        }
        if(faults.other) {
            s.addReport(ReportLevel.ERROR, "Fault type is 'other'. Hope for the best!");
        }
        if(faults.sensor) {
            s.addReport(ReportLevel.ERROR, "Sensor fault");
        }
        if(faults.temperature) {
            s.addReport(ReportLevel.ERROR, "Temperature fault");
        }

        Warnings warnings = motor.getWarnings();
        if(warnings.brownout) {
            s.addReport(ReportLevel.WARNING, "Brownout detected");
        }
        if (warnings.escEeprom) {
            s.addReport(ReportLevel.WARNING, "Escape Eeprom. Cannot write to internal memory. (Why is only a warning)");
        }
        if (warnings.extEeprom) {
            s.addReport(ReportLevel.WARNING, "Exit Eeprom. Cannot write to internal memory. (Why is only a warning)");
        }
        if (warnings.hasReset) {
            s.addReport(ReportLevel.WARNING, "Has Reset");
        }
        if (warnings.other) {
            s.addReport(ReportLevel.WARNING, "Other. Warning message sold seperately");
        }
        if (warnings.overcurrent) {
            s.addReport(ReportLevel.WARNING, "Overcurrent");
        }
        if (warnings.sensor) {
            s.addReport(ReportLevel.WARNING, "Sensor problem");
        }
        if (warnings.stall) {
            s.addReport(ReportLevel.WARNING, "Motor stall detected");
        }

        return s;
    }
}
