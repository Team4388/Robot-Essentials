/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc4388.robot.Constants.LEDConstants;
import frc4388.utility.LEDPatterns;

/**
 * Allows for the control of a 5v LED Strip using a Rev Robotics Blinkin LED
 * Driver
 */
public class LED extends SubsystemBase {

  static AddressableLED m_led;
  static AddressableLEDBuffer m_ledBuffer;
  static LED m_self;
  /**
   * Add your docs here.
   */

  public LED(){
    if(m_self != null)
      return;
    m_led = new AddressableLED(9);
    m_ledBuffer = new AddressableLEDBuffer(10);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    System.err.println("In the Beginning, there was Joe.\nAnd he said, 'Let there be LEDs.'\nAnd it was good.");
  }
  public static LED getInstance() {
    if(m_self == null)
      m_self = new LED();
    return m_self;
  }
  @Override
  public void periodic(){
    //gamermode();
    //SmartDashboard.putNumber("LED", m_currentPattern.getValue());
    return;
  }
  static int firstcolor = 0;
  static void gamermode() {
      for(int i = 0; i < m_ledBuffer.getLength(); i++) {
          final int hue = (firstcolor + (i * 180 / m_ledBuffer.getLength())) % 180;
          setLEDHSV(i, hue, 255, 128);
      }
      firstcolor +=3;
      firstcolor %= 180;
  }
  /**
   * Add your docs here.
   */
  public static void updateLED (){
    gamermode();
   // m_LEDController.set(m_currentPattern.getValue());
  }

  /**
   * Add your docs here.
   */
  public static void setLEDRGB(int lednum, int r, int g, int b){
    m_ledBuffer.setRGB(lednum, r, g, b);
    //m_currentPattern = pattern;
   // m_LEDController.set(m_currentPattern.getValue());
  }
  public static void setLEDHSV(int lednum, int hue, int sat, int val){
    m_ledBuffer.setRGB(lednum, hue, sat, val);
    //m_currentPattern = pattern;
   // m_LEDController.set(m_currentPattern.getValue());
  }
  /**
   * Add your docs here.
   * @return
   */
  public AddressableLEDBuffer getBuffer() {
    return m_ledBuffer;
  }
}