package frc4388.utility.status;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

public class Status {
    public enum ReportLevel {
        INFO,
        WARNING,
        ERROR
    }

    public class Report {
        public ReportLevel reportLevel;
        public String description;

        @Override
        public String toString() {
            return this.reportLevel.name() + ": " + this.description;
        }
    }

    public List<Report> reports;

    public Status() {
        this.reports = new ArrayList<>();
    }

    public void addReport(ReportLevel level, String description) {
        Report r = new Report();
        r.reportLevel = level;
        r.description = description;
        this.reports.add(r);
    }

    private String printStatusCode(StatusCode status){
        return status.getName() + " (" + status.value + ")";
    }

    public void diagnoseHardwareCTRE(String deviceName, TalonFX motor) {
        addReport(ReportLevel.ERROR, deviceName + " Motor (TalonFX) Alive?: " + (motor.isAlive() ? "Alive." : "Dead!"));



        if (motor.isAlive()) addReport(ReportLevel.INFO, deviceName + " Motor (TalonFX) Alive?: Alive.");
        else addReport(ReportLevel.ERROR, deviceName + " Motor (TalonFX) Alive?: Dead!");
    }

    public void diagnoseHardwareCTRE(String deviceName, CANcoder coder) {
        // Because the Cancoder has no method to check its alive, we send it a empty control which it should return a zero when it gets the control.
        // If its not zero, that means that most likely that it had some communication error, I.e. It actually is powered off or not connected at all.
        // TODO: validate that a CANCoder can actually do `EmptyControl`s
        StatusCode status = coder.setControl(new EmptyControl()); 
        if (status.value == 0) addReport(ReportLevel.INFO, deviceName + " Cancoder Alive?: Alive. " + printStatusCode(status));
        else addReport(ReportLevel.ERROR, deviceName + " Cancoder Alive?: Dead! " + printStatusCode(status));

        
        
        // StatusSignal<MagnetHealthValue> -> MagnetHealthValue -> int
        int coderMagHealth = coder.getMagnetHealth().getValue().value;
        if (coderMagHealth == 3) addReport(ReportLevel.INFO, deviceName + " Cancoder Magnet Strength?: Ideal."); // why is 3 the 'good value'?
        if (coderMagHealth == 2) addReport(ReportLevel.WARNING, deviceName + " Cancoder Magnet Strength?: Subpar.");
        if (coderMagHealth == 1) addReport(ReportLevel.ERROR, deviceName + " Cancoder Magnet Strength?: Too Close or Far!");
        if (coderMagHealth == 0) addReport(ReportLevel.ERROR, deviceName + " Cancoder Magnet Strength?: Unkown!");
    }

    public void diagnoseHardwareCTRE(String deviceName, Pigeon2 pigeon) {
        // Because the Pigeon has no method to check its alive, we send it a empty control which it should return a zero when it gets the control.
        // If its not zero, that means that most likely that it had some communication error, I.e. It actually is powered off or not connected at all.
        // TODO: validate that a Pigeon2 can actually do `EmptyControl`s
        StatusCode status = pigeon.setControl(new EmptyControl()); 
        if (status.value == 0) addReport(ReportLevel.INFO, deviceName + " Pigeon2 Alive?: Alive. " + printStatusCode(status));
        else addReport(ReportLevel.ERROR, deviceName + " Pigeon2 Alive?: Dead! " + printStatusCode(status));
    }
    
    public boolean hasReport() {
        return reports.size() == 0;
    }
}
