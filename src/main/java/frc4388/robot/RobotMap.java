/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 * 
 * @deprecated
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  /* Xbox Controllers */
  public static final int XBOX_DRIVER_ID = 0;
  public static final int XBOX_OPERATOR_ID = 1;

  /* Blinkin LED Strip */
  public static final int LED_SPARK_ID = 0;

  /* Drive Train */
  public static final int DRIVE_LEFT_FRONT_CAN_ID = 2;
  public static final int DRIVE_RIGHT_FRONT_CAN_ID = 4;
	public static final int DRIVE_LEFT_BACK_CAN_ID = 3;
	public static final int DRIVE_RIGHT_BACK_CAN_ID = 5;
}
