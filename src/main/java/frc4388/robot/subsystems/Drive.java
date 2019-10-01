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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc4388.robot.RobotMap;
import frc4388.robot.OI;
import frc4388.robot.Robot;
import frc4388.utility.controller.XboxController;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  /* false for talon drivetrain, true for spark drivetrain */
  public static boolean isSpark = true;

  /* talons */
  public static WPI_TalonSRX m_leftFrontMotorTalon = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_FRONT_CAN_ID);
  public static WPI_TalonSRX m_rightFrontMotorTalon = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_FRONT_CAN_ID);
  public static WPI_TalonSRX m_leftBackMotorTalon = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_BACK_CAN_ID);
  public static WPI_TalonSRX m_rightBackMotorTalon = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_BACK_CAN_ID);

  /* sparks */
  public static CANSparkMax m_leftFrontMotorSpark = new CANSparkMax(RobotMap.DRIVE_LEFT_FRONT_CAN_ID, MotorType.kBrushless);
  public static CANSparkMax m_rightFrontMotorSpark = new CANSparkMax(RobotMap.DRIVE_RIGHT_FRONT_CAN_ID, MotorType.kBrushless);
  public static CANSparkMax m_leftBackMotorSpark = new CANSparkMax(RobotMap.DRIVE_LEFT_BACK_CAN_ID, MotorType.kBrushless);
  public static CANSparkMax m_rightBackMotorSpark = new CANSparkMax(RobotMap.DRIVE_RIGHT_BACK_CAN_ID, MotorType.kBrushless);

  /* drivetrain */
  public static DifferentialDrive m_driveTrain;
  

  /* inputs */
  private static double m_inputMove, m_inputSteer;

  public Drive() {
    if (isSpark) {
      initSpark();
    } 
    else {
      initTalon();
    }
  }

  @Override
  public void periodic() {
    m_inputMove = OI.getInstance().getDriverController().getLeftYAxis();
    m_inputSteer = -(OI.getInstance().getDriverController().getRightXAxis());

    m_driveTrain.arcadeDrive(m_inputMove, m_inputSteer);
  }

  private void initTalon() {
    /* create drivetrain */
    m_driveTrain = new DifferentialDrive(m_leftFrontMotorTalon, m_rightFrontMotorTalon);

    /* factory default values */
    m_leftFrontMotorTalon.configFactoryDefault();
    m_rightFrontMotorTalon.configFactoryDefault();
    m_leftBackMotorTalon.configFactoryDefault();
    m_rightBackMotorTalon.configFactoryDefault();

    /* set back motors as followers */
    m_leftBackMotorTalon.follow(m_leftFrontMotorTalon);
    m_rightBackMotorTalon.follow(m_rightFrontMotorTalon);

    /* set neutral mode */
    m_leftFrontMotorTalon.setNeutralMode(NeutralMode.Brake);
    m_rightFrontMotorTalon.setNeutralMode(NeutralMode.Brake);
    m_leftFrontMotorTalon.setNeutralMode(NeutralMode.Brake);
    m_rightFrontMotorTalon.setNeutralMode(NeutralMode.Brake);

    /* flip input so forward becomes back, etc */
    m_leftFrontMotorTalon.setInverted(false);
    m_rightFrontMotorTalon.setInverted(false);
    m_leftBackMotorTalon.setInverted(InvertType.FollowMaster);
    m_rightBackMotorTalon.setInverted(InvertType.FollowMaster);
  }

  private void initSpark(){

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
