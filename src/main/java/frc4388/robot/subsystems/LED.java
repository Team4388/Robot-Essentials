/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.constants.Constants.LEDConstants;
import frc4388.utility.status.Status;
import frc4388.utility.status.FaultReporter;
import frc4388.utility.status.Queryable;
import frc4388.utility.status.Status.ReportLevel;
import frc4388.utility.structs.LEDPatterns;

/**
 * Allows for the control of a 5v LED Strip using a Rev Robotics Blinkin LED
 * Driver
 */
public class LED extends SubsystemBase implements Queryable {
  public LED() {
    FaultReporter.register(this);
  }

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

  @AutoLogOutput
  public String state() {
    return mode.getClass().toString();
  }

  @Override
  public String getName() {
    return "LEDs";
  }

  @Override
  public Status diagnosticStatus() {
    Status status = new Status();

    if(!LEDController.isAlive())
      status.addReport(ReportLevel.ERROR, "LED is DISCONNECTED");
    
    status.addReport(ReportLevel.INFO, "LED Mode: " + mode.name());

    return status;
  }
  

}