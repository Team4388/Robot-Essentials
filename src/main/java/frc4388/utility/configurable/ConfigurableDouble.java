package frc4388.utility.configurable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ConfigurableDouble {
    private double defualtValue;
    private String name;

    /**
     * Creates an new ConfigurableDouble through Smart Dashboard.
     * @param name the name of the Smart Dashboard key.
     * @param defualtValue the initilization value
     */
    public ConfigurableDouble(String name, double defualtValue) {
        this.name = name;
        this.defualtValue = defualtValue;
        SmartDashboard.putNumber(name, defualtValue);
    }

    public double get() {
        return SmartDashboard.getNumber(name, defualtValue);
    }
}
