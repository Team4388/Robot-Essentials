package frc4388.utility.configurable;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ConfigurableString {
    private String defualtValue;
    private String name;

    /**
     * Creates an new ConfigurableString through Smart Dashboard.
     * @param name the name of the Smart Dashboard key.
     * @param defualtValue the initilization value
     */
    public ConfigurableString(String name, String defualtValue) {
        this.name = name;
        this.defualtValue = defualtValue;
        SmartDashboard.putString(name, defualtValue);
    }

    public String get() {
        return SmartDashboard.getString(name, defualtValue);
    }
}
