package frc4388.utility;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Subsystem extends SubsystemBase {
    public void Log(String str) {
        System.out.println(getSubsystemName() + " - " + str);
    }

    // Get name of subsystem, for use in log.
    public abstract String getSubsystemName();
    // Get what the subystem is currently doing, such as "Shooter spun up"
    public abstract Status queryStatus();
    // Proactivly search for any errors in each subsystem
    public abstract Status diagnosticStatus(); 
}
