// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.commands.alignment;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc4388.robot.subsystems.lidar.LiDAR;
import frc4388.robot.subsystems.swerve.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LidarAlign extends Command {
  private SwerveDrive swerveDrive;
  private LiDAR lidar;  

  private int currentFinderTick;
  // private int tickFoundPipe;
  private boolean foundReef;
  private boolean headedRight;
  private double speed;
  private int bounces;
  private double additionalDistance = 0;
  // private final boolean constructedHeadedRight;

  /** Creates a new LidarAlign. */
  public LidarAlign(SwerveDrive swerveDrive, LiDAR lidar) {//, boolean headedRight) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.swerveDrive = swerveDrive;
    this.lidar = lidar;

    addRequirements(swerveDrive, lidar);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.currentFinderTick = 0;
    this.speed = 0.4; // TODO: find good speed for this
    this.foundReef = false;
    this.headedRight = (DriveToReef.tagRelativeXError < 0);
    this.additionalDistance = 0;
    this.bounces = 0;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lidar.withinDistance()) {
      swerveDrive.softStop(); 
      foundReef = true;
      return;
    }

    if (currentFinderTick > (15 + additionalDistance)) { //arbutrary threshhold for now.
      headedRight = !headedRight;
      currentFinderTick *= -1;
      bounces++;
      additionalDistance += 5;
      if (bounces == 5) return;
    }
    double currentHeading = (swerveDrive.getGyroAngle() * 180) / Math.PI;
    double relAngle = (Math.round(currentHeading / 60.d) * 60); // Relative driving to the side of the reef
    SmartDashboard.putNumber("Rel Angle", relAngle);
    SmartDashboard.putNumber("heading", currentHeading);
    if (!headedRight) {
      swerveDrive.driveRelativeLockedAngle(new Translation2d(0, -speed), Rotation2d.fromDegrees(relAngle));
    } else {
      swerveDrive.driveRelativeLockedAngle(new Translation2d(0, speed), Rotation2d.fromDegrees(relAngle));
    }

    currentFinderTick++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (lidar.getDistance() < 4) {
      swerveDrive.stopModules();
      return true;
    } else if (foundReef && lidar.withinDistance()) { // spot on
      swerveDrive.stopModules();
      return true;
    } else if (foundReef && !lidar.withinDistance()) { // over shot
      speed = speed / 2;
      headedRight = !headedRight;
      currentFinderTick = 0;
      foundReef = false;
      return false;
    } else if (bounces >= 3) {
      swerveDrive.stopModules();
      return true;
    } else {
      return false;
    }
  }
}
