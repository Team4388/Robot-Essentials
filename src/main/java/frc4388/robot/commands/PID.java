// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc4388.utility.Gains;

public abstract class PID extends Command {
	protected Gains  gains;
	private   double output    = 0;
	private   double tolerance = 0;

	/** Creates a new PelvicInflammatoryDisease. */
	public PID(double kp, double ki, double kd, double kf, double tolerance) {
		gains          = new Gains(kp, ki, kd, kf, 0);
		this.tolerance = tolerance;
	}

	public PID(Gains gains, double tolerance) {
		this.gains     = gains;
		this.tolerance = tolerance;
	}

	/** produces the error from the setpoint */
	public abstract double getError();
	
	/** todo: javadoc */
	public abstract void runWithOutput(double output);

	// Called when the command is initially scheduled.
	@Override 
	public final void initialize() {
		output = 0;
	}

	private double prevError, cumError = 0;
	
	// Called every time the scheduler runs while the command is scheduled.
	@Override 
	public final void execute() {
		double error = getError();
		cumError += error * .02; // 20 ms
		double delta = error - prevError;

		output = error * gains.kP;
		output += cumError * gains.kI;
		output += delta * gains.kD;
		output += gains.kF;

		runWithOutput(output);
	}

	// Returns true when the command should end.
	@Override
	public final boolean isFinished() {
		return Math.abs(getError()) < tolerance;
	}
}
