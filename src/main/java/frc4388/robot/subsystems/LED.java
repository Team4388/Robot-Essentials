/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import frc4388.robot.RobotMap;
import frc4388.robot.commands.LED.UpdateLED;
import frc4388.robot.constants.LEDPatterns;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Allows for the control of a 5v LED Strip using a Rev Robotics Blinkin LED Driver
 */
 public class LED extends Subsystem {

  public static float currentLED;
  public static Spark LEDController;

  public LED(){
    LEDController = new Spark(RobotMap.LED_SPARK_ID);
    setPattern(LEDPatterns.FOREST_WAVES);
    LEDController.set(currentLED);
    System.err.println("In the Beginning, there was Joe.\nAnd he said, 'Let there be LEDs.'\nAnd it was good.");
  }

  public void updateLED(){
    LEDController.set(currentLED);
  }

  public void setPattern(LEDPatterns pattern){
    currentLED = pattern.getValue();
    LEDController.set(currentLED);
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("LED", currentLED);
  }

  @Override
	public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new UpdateLED());
  }
}