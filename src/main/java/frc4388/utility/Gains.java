// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

/** Add your docs here. */
public class Gains {
    public double kP;
    public double kI;
    public double kD;
    public double kF;
    public int kIZone;
    public double kPeakOutput;
    public double kMaxOutput;
    public double kMinOutput;

    /**
     * Creates Gains object for PIDs
     * @param kP The P value.
     * @param kI The I value.
     * @param kD The D value.
     * @param kF The F value.
     * @param kIZone The zone of the I value.
     * @param kPeakOutput The peak output setting the motors to run the gains at, in both forward and reverse directions. By default 1.0.
     */
    public Gains(double kP, double kI, double kD, double kF, int kIZone, double kPeakOutput) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kIZone = kIZone;
        this.kPeakOutput = kPeakOutput;
        this.kMaxOutput = kPeakOutput;
        this.kMinOutput = -kPeakOutput;
    }

    /**
     * Creates Gains object for PIDs
     * @param kP The P value.
     * @param kI The I value.
     * @param kD The D value.
     * @param kF The F value.
     * @param kIZone The zone of the I value.
     */
    public Gains(double kP, double kI, double kD, double kF, int kIZone) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kIZone = kIZone;
        this.kPeakOutput = 1.0;
        this.kMaxOutput = 1.0;
        this.kMinOutput = -1.0;
    }

    public Gains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Creates Gains object for PIDs
     * @param kP The P value.
     * @param kI The I value.
     * @param kD The D value.
     * @param kF The F value.
     * @param kIZone The zone of the I value.
     * @param kMinOutput The lowest output setting to run the gains at, usually in the reverse direction. By default -1.0.
     * @param kMaxOutput The highest output setting to run the gains at, usually in the forward direction. By default 1.0.
     */
    public Gains(double kP, double kI, double kD, double kF, int kIZone, double kMaxOutput, double kMinOutput) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.kIZone = kIZone;
        this.kMaxOutput = kMaxOutput;
        this.kMinOutput = kMinOutput;
        this.kPeakOutput = (Math.abs(kMinOutput) > Math.abs(kMaxOutput)) ? Math.abs(kMinOutput) : Math.abs(kMaxOutput);
    }    
}