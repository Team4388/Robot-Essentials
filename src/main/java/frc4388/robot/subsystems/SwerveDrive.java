// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.RobotGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive extends SubsystemBase {
  
  private SwerveModule leftFront;
  private SwerveModule rightFront;
  private SwerveModule leftBack;
  private SwerveModule rightBack;

  private SwerveModule[] modules;

  private Translation2d leftFrontLocation = new Translation2d(Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d rightFrontLocation = new Translation2d(Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), -Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d leftBackLocation = new Translation2d(-Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  private Translation2d rightBackLocation = new Translation2d(-Units.inchesToMeters(SwerveDriveConstants.HALF_HEIGHT), -Units.inchesToMeters(SwerveDriveConstants.HALF_WIDTH));
  
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation);

  private RobotGyro gyro;

  public double speedAdjust = SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW; // * slow by default
  
  public double rotTarget = 0.0;
  public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  /** Creates a new SwerveDrive. */
  public SwerveDrive(SwerveModule leftFront, SwerveModule rightFront, SwerveModule leftBack, SwerveModule rightBack, RobotGyro gyro) {
    this.leftFront = leftFront;
    this.rightFront = rightFront;
    this.leftBack = leftBack;
    this.rightBack = rightBack;
    
    this.gyro = gyro;

    this.modules = new SwerveModule[] {this.leftFront, this.rightFront, this.leftBack, this.rightBack};
  }

  boolean stopped = false;
  public void driveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {
    if (fieldRelative) {

      double rot = 0;
      
      if (rightStick.getNorm() > 0.05) {
        rotTarget = gyro.getAngle();
        rot = rightStick.getX() * SwerveDriveConstants.ROTATION_SPEED;
        SmartDashboard.putBoolean("drift correction", false);
        stopped = false;
      } else if(leftStick.getNorm() > 0.05) {
        if (!stopped) {
          stopModules();
          stopped = true;
        }

        SmartDashboard.putBoolean("drift correction", true);
        rot = ((rotTarget - gyro.getAngle()) / 360) * SwerveDriveConstants.ROT_CORRECTION_SPEED;

      }

      // Use the left joystick to set speed. Apply a cubic curve and the set max speed.
      Translation2d speed = leftStick.times(leftStick.getNorm() * speedAdjust);
      // Translation2d cubedSpeed = new Translation2d(Math.pow(speed.getX(), 3.00), Math.pow(speed.getY(), 3.00));

      // Convert field-relative speeds to robot-relative speeds.
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-1 * speed.getX(), speed.getY(), rightStick.getX() * SwerveDriveConstants.ROTATION_SPEED, gyro.getRotation2d().times(-1));
    } else {
      // Create robot-relative speeds.
      chassisSpeeds = new ChassisSpeeds(-1 * leftStick.getX(), leftStick.getY(), rightStick.getX() * SwerveDriveConstants.ROTATION_SPEED);
    }
    setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  /**
   * Set each module of the swerve drive to the corresponding desired state.
   * @param desiredStates Array of module states to set.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SECOND));
    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = desiredStates[i];
      module.setDesiredState(state);
    }
  }

  public boolean rotateToTarget(double angle) {
    double currentAngle = getGyroAngle();
    double error = angle - currentAngle;

    driveWithInput(new Translation2d(0, 0), new Translation2d(error / Math.abs(error) * 0.3, 0), true);

    if (Math.abs(angle - getGyroAngle()) < 5.0) {
      return true;
    }

    return false;
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  public void resetGyro() {
    gyro.reset();
    rotTarget = 0.0;
  }
  
  public void stopModules() {
    for (SwerveModule module : this.modules) {
      module.stop();
    }
  }

  public SwerveDriveKinematics getKinematics() {
    return this.kinematics;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    SmartDashboard.putNumber("Gyro", getGyroAngle());
  }

  public void shiftDown() {
    if (Math.abs(this.speedAdjust - SwerveDriveConstants.SLOW_SPEED) < .01) {
      
    } else if (Math.abs(this.speedAdjust - SwerveDriveConstants.FAST_SPEED) < .01) {
      this.speedAdjust = SwerveDriveConstants.SLOW_SPEED;
    } else {
      this.speedAdjust = SwerveDriveConstants.FAST_SPEED;
    }
  }

  public void setToSlow() {
    this.speedAdjust = SwerveDriveConstants.SLOW_SPEED;
    System.out.println("SLOW");
    System.out.println("SLOW");
    System.out.println("SLOW");
    System.out.println("SLOW");
    System.out.println("SLOW");
  }

  public void setToFast() {
    this.speedAdjust = SwerveDriveConstants.FAST_SPEED;
    System.out.println("FAST");
    System.out.println("FAST");
    System.out.println("FAST");
    System.out.println("FAST");
    System.out.println("FAST");
  }

  public void setToTurbo() {
    this.speedAdjust = SwerveDriveConstants.TURBO_SPEED;
    System.out.println("TURBO");
    System.out.println("TURBO");
    System.out.println("TURBO");
    System.out.println("TURBO");
    System.out.println("TURBO");
  }

  public void shiftUp() {
    if (Math.abs(this.speedAdjust - SwerveDriveConstants.SLOW_SPEED) < .01) {
      this.speedAdjust = SwerveDriveConstants.FAST_SPEED;
    } else if (Math.abs(this.speedAdjust - SwerveDriveConstants.FAST_SPEED) < .01) {
      this.speedAdjust = SwerveDriveConstants.TURBO_SPEED;
    } else {
      
    }
  }

  public void toggleGear(double angle) {
    if (Math.abs(this.speedAdjust - SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW) < .01 && Math.abs(angle) < 10) {
      this.speedAdjust = SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_FAST;
      SwerveDriveConstants.ROT_CORRECTION_SPEED = SwerveDriveConstants.CORRECTION_MIN;
    } else {
      this.speedAdjust = SwerveDriveConstants.Conversions.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
      SwerveDriveConstants.ROT_CORRECTION_SPEED = SwerveDriveConstants.CORRECTION_MIN;
    }
  }

}
