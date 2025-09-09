// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility.compute;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.ArrayList;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Reboot persistant Trims.
 * @author Zachary Wilke
 */
public class Trim {
    private static ArrayList<Trim> trims = new ArrayList<Trim>();
    private static ShuffleboardTab trimTab = Shuffleboard.getTab("Trims");

    private String trimName; 
    private double upperBound;
    private double lowerBound;
    private double step;

    private boolean modified = false;
    private double currentValue;
    private boolean persistant = false;

    private GenericEntry trimElement = null;

    /**
     * Creates a variably Trim with a given name, upper and lower bounds, step size and intial value
     * @param trimName please keep the trim name without special symbols
     * @param upperBound the upper limit inclusive
     * @param lowerBound the lower limit inclusive
     * @param step the step size
     * @param inital the inital value, will get overridden if the persistant trim exists on disk.
     * @param persistnat Weather the trim is persistant or not
     */
    public Trim(String trimName, double upperBound, double lowerBound, double step, double inital, boolean persistant) {
        this.trimName = trimName;
        this.upperBound = upperBound;
        this.lowerBound = lowerBound;
        this.step = step;
        this.persistant = persistant;
        currentValue = inital;
        load();
        trimElement = trimTab.add(trimName, currentValue).getEntry();

        trims.add(this);
    }

    /**
     * Creates a non-Trim with a given name, upper and lower bounds, step size and intial value
     * @param trimName please keep the trim name without special symbols
     * @param upperBound the upper limit inclusive
     * @param lowerBound the lower limit inclusive
     * @param step the step size
     * @param inital the inital value, will get overridden if the persistant trim exists on disk.
     */
    public Trim(String trimName, double upperBound, double lowerBound, double step, double inital) {
        this.trimName = trimName;
        this.upperBound = upperBound;
        this.lowerBound = lowerBound;
        this.step = step;
        currentValue = inital;
        load();
        trimElement = trimTab.add(trimName, currentValue).getEntry();

        trims.add(this);
    }

    private void clampModify() {
        currentValue = Math.min(upperBound, Math.max(currentValue, lowerBound));
        if (trimElement != null) 
            trimElement.setValue(currentValue);
        modified = true;
    }

    public void stepUp() {
        this.currentValue += step;
        clampModify();
    }

    public void stepDown() {
        this.currentValue -= step;
        clampModify();
    }
    
    public void set(double value) {
        this.currentValue = value;
        clampModify();
    }

    public double get() {
        return this.currentValue;
    }

    public boolean isModified() {
        return modified;
    }

    public boolean load() {
        if(!persistant)
            return false;

        try (FileInputStream stream = new FileInputStream("/home/lvuser/trims/" + trimName)) {
            double fileValue = DataUtils.byteArrayToDouble(stream.readNBytes(8));
            currentValue = fileValue;
            clampModify();
            modified = false;
            if (fileValue != currentValue) {
                System.out.println("TRIMS: Loaded trim `" + trimName + "` has a value that is higher than or less than the bounds set for the trim, clamping...");
                modified = true;
            }
            return true;
        } catch (Exception e) {
            // e.printStackTrace();
            System.out.println("TRIMS: Unable to read trim file `" + trimName + "`, using current value...");
            return false;
        }
        
    }

    public void dump() {
        try (FileOutputStream stream = new FileOutputStream("/home/lvuser/trims/" + trimName)) {
            stream.write(DataUtils.doubleToByteArray(currentValue));
            modified = false;
        } catch (Exception e) {
            // e.printStackTrace();
            System.out.println("TRIMS: Unable to write to trim file `" + trimName + "`!?!");
        }
    }

    public static void dumpAll() {
        for (int i = 0; i < trims.size(); i++) {
            Trim trim = trims.get(i);
            if (trim.isModified()) 
                trim.dump();
        }
    }
}
