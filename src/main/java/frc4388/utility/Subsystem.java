package frc4388.utility;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {
    public static List<Subsystem> subsystems = new ArrayList<>();

    public Subsystem(){
        subsystems.add(this);
    }

    public void Log(String str) {
        System.out.println(getSubsystemName() + " - " + str);
    }

    // Get name of subsystem, for use in log.
    public abstract String getSubsystemName();
    // Get what the subystem is currently doing, such as "Shooter spun up". This should post to SmartDashboard
    public abstract void queryStatus();
    // Proactivly search for any errors in each subsystem
    public abstract Status diagnosticStatus(); 
}
