package frc4388.utility.status;

import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.hardware.TalonFX;

import frc4388.utility.status.Status.ReportLevel;

public class FaultTalonFX implements Queryable {
    private String name;
    private TalonFX motor;

    public static void addDevice(TalonFX motor, String name) {
        FaultTalonFX p = new FaultTalonFX();

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


        boolean debounceBad = !QueryUtils.isDebounceOk(motor.getSupplyVoltage());
        boolean emptyControlBad = motor.setControl(new EmptyControl()).value != 0;

        if(debounceBad || emptyControlBad) {
            s.addReport(ReportLevel.ERROR, "device is unreachable - Failed" + 
            (debounceBad ? " Failed debounce test" : "") + 
            (emptyControlBad ? " Failed empty control test" : "")
            );
        }


        s.addReport(ReportLevel.INFO, "Voltage: " + motor.getSupplyVoltage());
        s.addReport(ReportLevel.INFO, "Current: " + motor.getSupplyCurrent());
        s.addReport(ReportLevel.INFO, "Device temp: " + motor.getDeviceTemp());
        s.addReport(ReportLevel.INFO, "Processor temp: " + motor.getProcessorTemp());
        s.addReport(ReportLevel.INFO, "Position: " + motor.getPosition());
        s.addReport(ReportLevel.INFO, "Velocity: " + motor.getVelocity());
        s.addReport(ReportLevel.INFO, "Acceleration: " + motor.getAcceleration());

        // faults<
        if (motor.getFault_BootDuringEnable().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Device booted while enabled");
        }
        if (motor.getFault_BridgeBrownout().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Bridge was disabled most likely due to supply voltage dropping too low");
        }
        if (motor.getFault_DeviceTemp().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Device temperature exceeded limit");
        }
        if (motor.getFault_FusedSensorOutOfSync().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Remote sensor is out of sync");
        }
        if (motor.getFault_Hardware().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Hardware failure detected");
        }
        if (motor.getFault_MissingDifferentialFX().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "The remote Talon used for differential control is not present");
        }
        if (motor.getFault_MissingHardLimitRemote().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR,"The remote limit switch device is not present");
        }
        if (motor.getFault_MissingSoftLimitRemote().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "The remote soft limit device is not present");
        }
        if (motor.getFault_OverSupplyV().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Supply voltage exceeded limit");
        }
        if (motor.getFault_ProcTemp().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Processor temperature exceeded limit");
        }
        if (motor.getFault_RemoteSensorDataInvalid().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "The remote sensor's data is no longer trusted");
        }
        if (motor.getFault_RemoteSensorPosOverflow().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "remote sensor position has overflowed");
        }
        if (motor.getFault_RemoteSensorReset().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "The remote sensor has reset");
        }
        if (motor.getFault_Undervoltage().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, " Device supply voltage near brownout");
        }
        if (motor.getFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Unlicensed feature in use");
        }
        if (motor.getFault_UnstableSupplyV().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Supply voltage is unstable");
        }
  

        // sticky faults
        if (motor.getStickyFault_BootDuringEnable().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Device booted while enabled");
        }
        if (motor.getStickyFault_BridgeBrownout().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Bridge was disabled most likely due to supply voltage dropping too low");
        }
        if (motor.getStickyFault_DeviceTemp().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Device temperature exceeded limit");
        }
        if (motor.getStickyFault_FusedSensorOutOfSync().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Remote sensor is out of sync");
        }
        if (motor.getStickyFault_Hardware().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Hardware failure detected");
        }
        if (motor.getStickyFault_MissingDifferentialFX().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] The remote Talon used for differential control is not present");
        }
        if (motor.getStickyFault_MissingHardLimitRemote().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] The remote limit switch device is not present");
        }
        if (motor.getStickyFault_MissingSoftLimitRemote().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] The remote soft limit device is not present");
        }
        if (motor.getStickyFault_OverSupplyV().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Supply voltage exceeded limit");
        }
        if (motor.getStickyFault_ProcTemp().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Processor temperature exceeded limit");
        }
        if (motor.getStickyFault_RemoteSensorDataInvalid().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] The remote sensor's data is no longer trusted");
        }
        if (motor.getStickyFault_RemoteSensorPosOverflow().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] The remote sensor position has overflowed");
        }
        if (motor.getStickyFault_RemoteSensorReset().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] The remote sensor has reset");
        }
        if (motor.getStickyFault_Undervoltage().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Device supply voltage near brownout");
        }
        if (motor.getStickyFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Unlicensed feature in use");
        }
        if (motor.getStickyFault_UnstableSupplyV().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Supply voltage is unstable");
        }


        return s;
    }
}
