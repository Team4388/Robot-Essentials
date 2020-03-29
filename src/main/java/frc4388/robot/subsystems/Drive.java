/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc4388.robot.Constants.DriveConstants;
import frc4388.utility.RobotGyro;
import frc4388.utility.RobotTime;

/**
 * Add your docs here.
 */
public class Drive extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private WPI_TalonSRX m_leftFrontMotor;
  private WPI_TalonSRX m_rightFrontMotor;
  private WPI_TalonSRX m_leftBackMotor;
  private WPI_TalonSRX m_rightBackMotor;
  private DifferentialDrive m_driveTrain;
  private RobotGyro m_gyro;

  /**
   * Add your docs here.
   */
  public Drive(WPI_TalonSRX leftFrontMotor, WPI_TalonSRX rightFrontMotor, WPI_TalonSRX leftBackMotor,
      WPI_TalonSRX rightBackMotor, RobotGyro gyro) {

    m_leftFrontMotor = leftFrontMotor;
    m_rightFrontMotor = rightFrontMotor;
    m_leftBackMotor = leftBackMotor;
    m_rightBackMotor = rightBackMotor;
    m_driveTrain = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);
    m_gyro = gyro;
  }

  @Override
  public void periodic() {
    m_gyro.updatePigeonDeltas();

    if (RobotTime.m_frameNumber % DriveConstants.SMARTDASHBOARD_UPDATE_FRAME == 0) {
      updateSmartDashboard();
    }
  }

  /**
   * Add your docs here.
   */
  public void driveWithInput(double move, double steer) {
    m_driveTrain.arcadeDrive(move, steer);
  }

  /**
   * Add your docs here.
   */
  private void updateSmartDashboard() {

    // Examples of the functionality of RobotGyro
    SmartDashboard.putBoolean("Is Gyro a Pigeon?", m_gyro.m_isGyroAPigeon);
    SmartDashboard.putNumber("Turn Rate", m_gyro.getRate());
    SmartDashboard.putNumber("Gyro Pitch", m_gyro.getPitch());
    SmartDashboard.putData(m_gyro);
  }
}
