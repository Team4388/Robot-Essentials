// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import frc4388.robot.commands.PID;
import frc4388.robot.subsystems.swerve.SwerveDrive;

public class RotateToAngle extends PID {

  SwerveDrive drive;
  double targetAngle;

  /** Creates a new RotateToAngle. */
  public RotateToAngle(SwerveDrive drive, double targetAngle) {
    super(0.3, 0.0, 0.0, 0.0, 1);
    
    this.drive = drive;
    this.targetAngle = targetAngle;

    addRequirements(drive);
  }

  @Override
  public double getError() {
    return targetAngle - drive.getGyroAngle();
  }

  @Override
  public void runWithOutput(double output) {
    drive.driveWithInput(new Translation2d(0.0, 0.0), new Translation2d(output / Math.abs(getError()), 0.0), true);    
  }
}
