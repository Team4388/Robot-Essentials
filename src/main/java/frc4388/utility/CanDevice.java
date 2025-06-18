package frc4388.utility;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.hal.can.CANJNI;
import edu.wpi.first.wpilibj.CAN;
import frc4388.utility.Status.Report;
import frc4388.utility.Status.ReportLevel;

public class CanDevice {
    public static List<CanDevice> devices = new ArrayList<>();

    public String name;
    public int id;
    
    public CanDevice(String name, int id) {
        this.name = name;
        this.id = id;

        devices.add(this);
    }


    private boolean isAlive() {
        return true; //TODO: Link this with Device Finder
    }

    public String getName() {
        return "CAN ID " + this.id + " ( " + this.name + " ) ";
    }

    public void Log(String str){
        System.out.println(getName() + " - " + str);
    }

    // public Status queryStatus() {
    //      Status s = new Status();

    //     s.addReport(ReportLevel.INFO, "TODO");

    //     return s;
    // }

    public Status diagnosticStatus() {
        Status s = new Status();
        //TODO
        s.addReport(ReportLevel.INFO, "Add CAN magic here");
        boolean isAlive = isAlive();
        s.addReport(isAlive ? ReportLevel.INFO : ReportLevel.ERROR, "Is Alive: " + isAlive);

        return s;
    }

    
}
