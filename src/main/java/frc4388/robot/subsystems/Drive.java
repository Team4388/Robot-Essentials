/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc4388.robot.RobotMap;
import frc4388.robot.Robot;
import frc4388.utility.controller.XboxController;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_TalonSRX m_leftFrontMotor = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_FRONT_CAN_ID);
  WPI_TalonSRX m_rightFrontMotor = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_FRONT_CAN_ID);
  WPI_TalonSRX m_leftBackMotor = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_BACK_CAN_ID);
  WPI_TalonSRX m_rightBackMotor = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_BACK_CAN_ID);

  DifferentialDrive m_driveTrain = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);

  public Drive(){
    /* factory default values */
    m_leftFrontMotor.configFactoryDefault();
    m_rightFrontMotor.configFactoryDefault();
    m_leftBackMotor.configFactoryDefault();
    m_rightBackMotor.configFactoryDefault();

    /* set back motors as followers */
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    /* set neutral mode */
    m_leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    m_rightFrontMotor.setNeutralMode(NeutralMode.Brake);
    m_leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    m_rightFrontMotor.setNeutralMode(NeutralMode.Brake);

    /* flip input so forward becomes back, etc */
    m_leftFrontMotor.setInverted(false);
    m_rightFrontMotor.setInverted(false);
    m_leftBackMotor.setInverted(InvertType.FollowMaster);
    m_rightBackMotor.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
