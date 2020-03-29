/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc4388.robot.Constants.LEDConstants;
import frc4388.utility.LEDPatterns;

/**
 * Allows for the control of a 5v LED Strip using a Rev Robotics Blinkin LED
 * Driver
 */
public class LED extends SubsystemBase {

  private float m_currentLED;
  private Spark m_LEDController;

  /**
   * Add your docs here.
   */
  public LED(Spark LEDController){
    m_LEDController = LEDController;
    setPattern(LEDConstants.DEFAULT_PATTERN);
    m_LEDController.set(m_currentLED);
    System.err.println("In the Beginning, there was Joe.\nAnd he said, 'Let there be LEDs.'\nAnd it was good.");
  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("LED", m_currentLED);
  }

  /**
   * Add your docs here.
   */
  public void updateLED(){
    m_LEDController.set(m_currentLED);
  }

  /**
   * Add your docs here.
   */
  public void setPattern(LEDPatterns pattern){
    m_currentLED = pattern.getValue();
    m_LEDController.set(m_currentLED);
  }
}