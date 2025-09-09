package frc4388.utility.status;

public interface Queryable {
    // Get name of subsystem, for use in log.
    String getName();
    // Proactivly search for any errors in each subsystem
    Status diagnosticStatus(); 
}
