/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc4388.robot.Constants.LEDConstants;
import frc4388.utility.LEDPatterns;
import frc4388.utility.Status;
import frc4388.utility.Subsystem;
import frc4388.utility.Status.ReportLevel;

/**
 * Allows for the control of a 5v LED Strip using a Rev Robotics Blinkin LED
 * Driver
 */
public class LED extends Subsystem {

  private static Spark LEDController = new Spark(LEDConstants.LED_SPARK_ID);
  private LEDPatterns mode = LEDConstants.DEFAULT_PATTERN;

  public void setMode(LEDPatterns pattern){
    this.mode = pattern;
  }

  @Override
  public void periodic() {
    update();
  }

  public void update() {
    if(!LEDController.isAlive() || LEDController.isSafetyEnabled()) return;

    if(DriverStation.isDisabled()){
      LEDController.set(LEDConstants.DEFAULT_PATTERN.getValue());
    }else
      LEDController.set(mode.getValue());
  }

  @Override
  public String getSubsystemName() {
    return "LEDs";
  }

  @Override
  public void queryStatus() {
    SmartDashboard.putString("LED status", mode.name());
  }

  @Override
  public Status diagnosticStatus() {
    Status status = new Status();

    if(LEDController.isAlive())
      status.addReport(ReportLevel.INFO, "LED is CONNECTED");
    else
      status.addReport(ReportLevel.ERROR, "LED is DISCONNECTED");
    
    status.addReport(ReportLevel.INFO, "LED Mode: " + mode.name());

    return status;
  }
  

}