package frc4388.utility.status;

import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc4388.utility.status.Status.ReportLevel;

public class FaultPidgeon2 implements Queryable {
    private String name;
    private Pigeon2 pigeon2;

    public static void addDevice(Pigeon2 pigeon2, String name) {
        FaultPidgeon2 p = new FaultPidgeon2();

        p.name = name;
        p.pigeon2 = pigeon2;

        FaultReporter.register(p);
    }

    @Override
    public String getName() {
        return name;
    }
    
    @Override
    public Status diagnosticStatus() {
        Status s = new Status();



        boolean debounceBad = !QueryUtils.isDebounceOk(pigeon2.getSupplyVoltage());
        boolean emptyControlBad = pigeon2.setControl(new EmptyControl()).value != 0;

        if(debounceBad || emptyControlBad) {
            s.addReport(ReportLevel.ERROR, "device is unreachable - Failed" + 
            (debounceBad ? " Failed debounce test" : "") + 
            (emptyControlBad ? " Failed empty control test" : "")
            );
        }


        s.addReport(ReportLevel.INFO, "Voltage: " + pigeon2.getSupplyVoltage());

        s.addReport(ReportLevel.INFO, "Pitch: " + pigeon2.getPitch());
        s.addReport(ReportLevel.INFO, "Yaw: " + pigeon2.getYaw());
        s.addReport(ReportLevel.INFO, "Roll: " + pigeon2.getRoll());

        s.addReport(ReportLevel.INFO, "Acceleration X: " + pigeon2.getAccelerationX());
        s.addReport(ReportLevel.INFO, "Acceleration Y: " + pigeon2.getAccelerationY());
        s.addReport(ReportLevel.INFO, "Acceleration Z: " + pigeon2.getAccelerationZ());

        s.addReport(ReportLevel.INFO, "Magnomiter X: " + pigeon2.getMagneticFieldX());
        s.addReport(ReportLevel.INFO, "Magnomiter Y: " + pigeon2.getMagneticFieldY());
        s.addReport(ReportLevel.INFO, "Magnomiter Z: " + pigeon2.getMagneticFieldZ());


        // faults
        if (pigeon2.getFault_BootDuringEnable().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Device booted while enabled");
        }
        if (pigeon2.getFault_BootIntoMotion().getValue() == Boolean.TRUE) {
        s.addReport(ReportLevel.ERROR, "Device booted while in motion");
        }
        if (pigeon2.getFault_BootupAccelerometer().getValue() == Boolean.TRUE) {
        s.addReport(ReportLevel.ERROR, "Accelerometer fault detected");
        }
        if (pigeon2.getFault_BootupGyroscope().getValue() == Boolean.TRUE) {
        s.addReport(ReportLevel.ERROR, "Gyro fault detected");
        }
        if (pigeon2.getFault_BootupMagnetometer().getValue() == Boolean.TRUE) {
        s.addReport(ReportLevel.ERROR, "Magnetometer fault detected");
        }
        if (pigeon2.getFault_DataAcquiredLate().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, 
                "Motion stack data acquisition slower than expected");
        }
        if (pigeon2.getFault_Hardware().getValue() == Boolean.TRUE) {
        s.addReport(ReportLevel.ERROR, "Hardware fault detected");
        }
        if (pigeon2.getFault_LoopTimeSlow().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, 
                "Motion stack loop time was slower than expected");
        }
        if (pigeon2.getFault_SaturatedAccelerometer().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, "Accelerometer values are saturated");
        }
        if (pigeon2.getFault_SaturatedGyroscope().getValue() == Boolean.TRUE) {
        s.addReport(ReportLevel.ERROR, "Gyro values are saturated");
        }
        if (pigeon2.getFault_SaturatedMagnetometer().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, "Magnetometer values are saturated");
        }
        if (pigeon2.getFault_Undervoltage().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, "Device supply voltage near brownout");
        }
        if (pigeon2.getFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
        s.addReport(ReportLevel.ERROR, "Unlicensed feature in use");
        }

        // sticky faults
        if (pigeon2.getStickyFault_BootDuringEnable().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, "[STICKY] Device booted while enabled");
        }
        if (pigeon2.getStickyFault_BootIntoMotion().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, "[STICKY] Device booted while in motion");
        }
        if (pigeon2.getStickyFault_BootupAccelerometer().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, "[STICKY] Accelerometer fault detected");
        }
        if (pigeon2.getStickyFault_BootupGyroscope().getValue() == Boolean.TRUE) {
        s.addReport(ReportLevel.ERROR, "[STICKY] Gyro fault detected");
        }
        if (pigeon2.getStickyFault_BootupMagnetometer().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, "[STICKY] Magnetometer fault detected");
        }
        if (pigeon2.getStickyFault_DataAcquiredLate().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, 
                String.format(
                    "[STICKY] Motion stack data acquisition slower than expected"));
        }
        if (pigeon2.getStickyFault_Hardware().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, "[STICKY] Hardware fault detected");
        }
        if (pigeon2.getStickyFault_LoopTimeSlow().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, 
                String.format(
                    "[STICKY] Motion stack loop time was slower than expected"));
        }
        if (pigeon2.getStickyFault_SaturatedAccelerometer().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, 
                "[STICKY] Accelerometer values are saturated");
        }
        if (pigeon2.getStickyFault_SaturatedGyroscope().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, "[STICKY] Gyro values are saturated");
        }
        if (pigeon2.getStickyFault_SaturatedMagnetometer().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, 
                "[STICKY] Magnetometer values are saturated");
        }
        if (pigeon2.getStickyFault_Undervoltage().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, 
                "[STICKY] Device supply voltage near brownout");
        }
        if (pigeon2.getStickyFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
        s.addReport(
            ReportLevel.ERROR, "[STICKY] Unlicensed feature in use");
        }
      
        return s;
    }
}
