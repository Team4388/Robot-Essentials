// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.utility;

/** Aarav's good units class (better than WPILib)
 * @author Aarav Shah */

public class RobotUnits {
  // constants

  // angle conversions
  public static double degreesToRadians(final double degrees) {return degrees * Math.PI / 180;}

  public static double radiansToDegrees(final double radians) {return radians / Math.PI * 180;}

  // falcon conversions
  public static double falconTicksToRotations(final double ticks) {return ticks / 2048;}

  public static double falconRotationsToTicks(final double rotations) {return rotations * 2048;}

  // distance conversions
  public static double metersToFeet(final double meters) {return meters * 3.28084;}

  public static double feetToMeters(final double feet) {return feet / 3.28084;}
}