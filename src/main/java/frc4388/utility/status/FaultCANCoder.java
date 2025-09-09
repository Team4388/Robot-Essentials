package frc4388.utility.status;

import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.hardware.CANcoder;

import frc4388.utility.status.Status.ReportLevel;

public class FaultCANCoder implements Queryable {
    private String name;
    private CANcoder cancoder;

    public static void addDevice(CANcoder cancoder, String name) {
        FaultCANCoder p = new FaultCANCoder();

        p.name = name;
        p.cancoder = cancoder;

        FaultReporter.register(p);
    }

    @Override
    public String getName() {
        return name;
    }
    
    @Override
    public Status diagnosticStatus() {
        Status s = new Status();

        boolean debounceBad = !QueryUtils.isDebounceOk(cancoder.getSupplyVoltage());
        boolean emptyControlBad = cancoder.setControl(new EmptyControl()).value != 0;

        if(debounceBad || emptyControlBad) {
            s.addReport(ReportLevel.ERROR, "device is unreachable - Failed" + 
            (debounceBad ? " Failed debounce test" : "") + 
            (emptyControlBad ? " Failed empty control test" : "")
            );
        }

        // faults
        if (cancoder.getFault_Hardware().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Hardware fault detected");
        }
        if (cancoder.getFault_BootDuringEnable().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Device booted while enabled");
        }
        if (cancoder.getFault_BadMagnet().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Bad magnet");
        }
        if (cancoder.getFault_Undervoltage().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Device supply voltage near brownout");
        }
        if (cancoder.getFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "Unlicensed feature in use");
        }
    
        // sticky faults
        if (cancoder.getStickyFault_Hardware().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Hardware fault detected");
        }
        if (cancoder.getStickyFault_BootDuringEnable().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Device booted while enabled");
        }
        if (cancoder.getStickyFault_BadMagnet().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Bad magnet");
        }
        if (cancoder.getStickyFault_Undervoltage().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Device supply voltage near brownout");
        }
        if (cancoder.getStickyFault_UnlicensedFeatureInUse().getValue() == Boolean.TRUE) {
            s.addReport(ReportLevel.ERROR, "[STICKY] Unlicensed feature in use");
        }

        return s;
    }
}
