// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.OIConstants;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;

public class SwerveDrive extends SubsystemBase {

  private SwerveModule m_leftFront;
  private SwerveModule m_leftBack;
  private SwerveModule m_rightFront;
  private SwerveModule m_rightBack;

  double halfWidth = SwerveDriveConstants.WIDTH / 2.d;
  double halfHeight = SwerveDriveConstants.HEIGHT / 2.d;

  public static Gains m_swerveGains = SwerveDriveConstants.SWERVE_GAINS;

  Translation2d m_frontLeftLocation = 
      new Translation2d(
          Units.inchesToMeters(halfHeight), 
          Units.inchesToMeters(halfWidth));
  Translation2d m_frontRightLocation =
       new Translation2d(
          Units.inchesToMeters(halfHeight), 
          Units.inchesToMeters(-halfWidth));
  Translation2d m_backLeftLocation = 
      new Translation2d(
          Units.inchesToMeters(-halfHeight), 
          Units.inchesToMeters(halfWidth));
  Translation2d m_backRightLocation = 
      new Translation2d(
          Units.inchesToMeters(-halfHeight), 
          Units.inchesToMeters(-halfWidth));

  public SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation,
      m_backLeftLocation, m_backRightLocation);

  public SwerveModule[] modules;
  public WPI_Pigeon2 m_gyro;

  public SwerveDriveOdometry m_odometry;
  // public SwerveDriveOdometry m_odometry;

  public double speedAdjust = SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
  public boolean ignoreAngles;
  public Rotation2d rotTarget = new Rotation2d();
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  private final Field2d m_field = new Field2d();

  public SwerveDrive(SwerveModule leftFront, SwerveModule leftBack, SwerveModule rightFront, SwerveModule rightBack,
      WPI_Pigeon2 gyro) {

    m_leftFront = leftFront;
    m_leftBack = leftBack;
    m_rightFront = rightFront;
    m_rightBack = rightBack;
    m_gyro = gyro;

    modules = new SwerveModule[] {m_leftFront, m_rightFront, m_leftBack, m_rightBack};
    
    // m_poseEstimator = new SwerveDrivePoseEstimator(
    //     getRegGyro(),//m_gyro.getRotation2d(),
    //     new Pose2d(),
    //     m_kinematics,
    //     VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(1)),  // TODO: tune
    //     VecBuilder.fill(Units.degreesToRadians(1)),            // TODO: tune
    //     VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(1))); // TODO: tune
    
    m_odometry = new SwerveDriveOdometry(m_kinematics, m_gyro.getRotation2d());

    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);
  }

  public void driveWithInput(double speedX, double speedY, double rot, boolean fieldRelative) {
    Translation2d speed = new Translation2d(speedX, speedY);
    driveWithInput(speed, rot, fieldRelative);
  }

  /**
   * Method to drive the robot using joystick info.
   * @link https://github.com/ZachOrr/MK3-Swerve-Example
   * @param speeds[0]     Speed of the robot in the x direction (forward).
   * @param speeds[1]     Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void driveWithInput(Translation2d speed, double rot, boolean fieldRelative) {
    ignoreAngles = (speed.getX() == 0) && (speed.getY() == 0) && (rot == 0);

    double mag = speed.getNorm();
    speed = speed.times(mag * speedAdjust);

    double xSpeedMetersPerSecond = speed.getX();
    double ySpeedMetersPerSecond = speed.getY();
    chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond,
            -rot * SwerveDriveConstants.ROTATION_SPEED * 2, new Rotation2d(-m_gyro.getRotation2d().getRadians() + (Math.PI*2) + (Math.PI /2)))
        : new ChassisSpeeds(ySpeedMetersPerSecond, -xSpeedMetersPerSecond,
            -rot * SwerveDriveConstants.ROTATION_SPEED * 2);
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(states);
  }

  public void driveWithInput(double leftX, double leftY, double rightX, double rightY, boolean fieldRelative) {
    Translation2d speed = new Translation2d(leftX, leftY);
    Translation2d head = new Translation2d(rightX, rightY);
    driveWithInput(speed, head, fieldRelative);
  }

  // new Rotation2d((360 - m_gyro.getRotation2d().getDegrees() + 90) * (Math.PI/180)))
  public void driveWithInput(Translation2d leftStick, Translation2d rightStick, boolean fieldRelative) {    
    ignoreAngles = leftStick.getX() == 0 && leftStick.getY() == 0 && rightStick.getX() == 0 && rightStick.getY() == 0;
    leftStick = leftStick.times(leftStick.getNorm() * speedAdjust);
    if (Math.abs(rightStick.getX()) > OIConstants.RIGHT_AXIS_DEADBAND || Math.abs(rightStick.getY()) > OIConstants.RIGHT_AXIS_DEADBAND)
      rotTarget = new Rotation2d(rightStick.getX(), -rightStick.getY()).minus(new Rotation2d(0,1));
    double rot = rotTarget.minus(m_gyro.getRotation2d()).getRadians();
    if (ignoreAngles) {
      rot = 0;
    }
    double xSpeedMetersPerSecond = leftStick.getX();
    double ySpeedMetersPerSecond = leftStick.getY();
    chassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond,
            rot * SwerveDriveConstants.ROTATION_SPEED * 2, new Rotation2d(-m_gyro.getRotation2d().getRadians() + (Math.PI*2) + (Math.PI /2)))
        : new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rightStick.getX() * SwerveDriveConstants.ROTATION_SPEED * 2);
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(
        chassisSpeeds);
    // if (ignoreAngles) {
    //   SwerveModuleState[] lockedStates = new SwerveModuleState[states.length];
    //   for (int i = 0; i < states.length; i ++) {
    //     SwerveModuleState state = states[i];
    //     lockedStates[i]= new SwerveModuleState(0, state.angle);
    //   }
    //   setModuleStates(lockedStates);
    // }
    setModuleStates(states);
    // SmartDashboard.putNumber("rot", rot);
    // SmartDashboard.putNumber("rotarget", rotTarget.getDegrees());
  }

  /**
   * Set each module of the swerve drive to the corresponding desired state.
   * 
   * @param desiredStates Array of module states to set.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
        Units.feetToMeters(SwerveDriveConstants.MAX_SPEED_FEET_PER_SEC));
    // int i = 2; {
    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = desiredStates[i];
      module.setDesiredState(state, ignoreAngles);
    }
    // modules[0].setDesiredState(desiredStates[0], false);
  }

  public void setModuleRotationsToAngle(double angle) {
    for (int i = 0; i < modules.length; i++) {
      SwerveModule module = modules[i];
      module.rotateToAngle(angle);
    }
  }

  @Override
  public void periodic() {
    updateOdometry();
    updateSmartDash();

    // SmartDashboard.putNumber("Pigeon getRotation2d", m_gyro.getRotation2d().getDegrees());
    // SmartDashboard.putNumber("Pigeon getAngle", m_gyro.getAngle());
    // SmartDashboard.putNumber("Pigeon Yaw", m_gyro.getYaw());
    // SmartDashboard.putNumber("Pigeon Yaw (0 to 360)", m_gyro.getYaw() % 360);

    m_field.setRobotPose(getOdometry());
    super.periodic();
  }

  private void updateSmartDash() {
    // odometry
    SmartDashboard.putNumber("Odometry: X", getOdometry().getX());
    SmartDashboard.putNumber("Odometry: Y", getOdometry().getY());
    SmartDashboard.putNumber("Odometry: Theta", getOdometry().getRotation().getDegrees());

    // chassis speeds
    // TODO: find the actual max velocity in m/s of the robot in fast mode to have accurate chassis speeds
    // SmartDashboard.putNumber("Chassis Vel: X", chassisSpeeds.vxMetersPerSecond);
    // SmartDashboard.putNumber("Chassis Vel: Y", chassisSpeeds.vyMetersPerSecond);
    // SmartDashboard.putNumber("Chassis Vel: ω", chassisSpeeds.omegaRadiansPerSecond);
  }

  /**
   * Gets the current chassis speeds in m/s and rad/s.
   * @return Current chassis speeds (vx, vy, ω)
   */
  public ChassisSpeeds getChassisSpeeds() {
    return chassisSpeeds;
  }

  /**
   * Gets the current pose of the robot.
   * 
   * @return Robot's current pose.
   */
  public Pose2d getOdometry() {
    // return m_odometry.getPoseMeters();
    return m_odometry.getPoseMeters();
    // return m_poseEstimator.getEstimatedPosition();
  }

  public Pose2d getAutoOdo() {
    Pose2d workingPose = getOdometry();
    return new Pose2d(-workingPose.getX(), workingPose.getY(), workingPose.getRotation());
  }

  /**
   * Gets the current gyro using regression formula.
   * 
   * @return Rotation2d object holding current gyro in radians
   */
  public Rotation2d getRegGyro() { 
    // * test chassis regression
    // double regCur = 0.6552670369 + m_gyro.getRotation2d().getDegrees() * 0.9926871527;
    // * new robot regression
    double regCur = 0.2507023948 + m_gyro.getRotation2d().getDegrees() * 0.999034743;
    return new Rotation2d(Math.toRadians(regCur));
  }

  /**
   * Resets the odometry of the robot to the given pose.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    Rotation2d actualDWI = new Rotation2d(-m_gyro.getRotation2d().getRadians() + (Math.PI*2)); //+ (Math.PI/2));
    Rotation2d actual =  new Rotation2d(m_gyro.getRotation2d().getRadians());

    SmartDashboard.putNumber("AUTO ACTUAL GYRO", actual.getDegrees());
    SmartDashboard.putNumber("AUTO DWI GYRO", actual.getDegrees());

    m_odometry.update( actual,//m_gyro.getRotation2d(),//new Rotation2d((2 * Math.PI) - getRegGyro().getRadians()),
                       modules[0].getState(),
                       modules[1].getState(),
                       modules[2].getState(),
                       modules[3].getState());
  }
  
  
  /**
   * Resets pigeon.
   */
  public void resetGyro() {
    m_gyro.reset();
    rotTarget = new Rotation2d(0);
  }

  /**
   * Stop all four swerve modules.
   */
  public void stopModules() {
    modules[0].stop();
    modules[1].stop();
    modules[2].stop();
    modules[3].stop();
  }

  /**
   * Switches speed modes.
   * 
   * @param shift True if fast mode, false if slow mode.
   */
  public void highSpeed(boolean shift) {
    if (shift) {
      speedAdjust = SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_FAST;
    } else {
      speedAdjust = SwerveDriveConstants.JOYSTICK_TO_METERS_PER_SECOND_SLOW;
    }
  }

  public double getCurrent(){
    return m_leftFront.getCurrent() + m_rightFront.getCurrent() + m_rightBack.getCurrent() + m_leftBack.getCurrent();
  }

  public double getVoltage(){
    return m_leftFront.getVoltage() + m_rightFront.getVoltage() + m_rightBack.getVoltage() + m_leftBack.getVoltage();
  }
}