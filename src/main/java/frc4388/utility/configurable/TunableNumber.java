package frc4388.utility.configurable;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class TunableNumber {
    private final String name;
    private final ShuffleboardTab tab;
    private final double defaultValue;
    private GenericEntry entry;

    public TunableNumber(String tabName, String name, double defaultValue) {
        this.name = name;
        this.tab = Shuffleboard.getTab(tabName);
        this.defaultValue = defaultValue;
        this.entry = tab.add(name, defaultValue)
                        .withWidget(BuiltInWidgets.kTextView) // Use a text box for editing
                        .getEntry();
    }   

    public double get() {
        return entry.getDouble(defaultValue);
    }  

    public void set(double value) {
        entry.setDouble(value);
    }
}