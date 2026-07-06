/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems.led;

import java.util.concurrent.TimeUnit;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
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
  private PWM m_pwm;

  public LED(int PWMport) {
    FaultReporter.register(this);
    m_pwm = new PWM(PWMport);
    
    m_pwm.setBoundsMicroseconds(2003, 1550, 1500, 1460, 999);
    m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
    m_pwm.setSpeed(0.0);
    m_pwm.setZeroLatch();
    update();
  }

  private LEDPatterns mode = LEDConstants.DEFAULT_PATTERN;

  public void setMode(LEDPatterns pattern){
    // Don't stall the main thread every time the setMode function is called
    if(this.mode != pattern) {
      this.mode = pattern;
      update();
    }
  }

  @AutoLogOutput
  public String getMode(){
    return mode.name();
  }

  public void update() {
    if(DriverStation.isDisabled()){
      m_pwm.setSpeed(LEDConstants.DEFAULT_PATTERN.getValue());
    }else
      m_pwm.setSpeed(mode.getValue());
  }

  // I freaking hate unmaintained code so much
  // https://github.com/REVrobotics/Blinkin-Firmware/pull/12
  // Turns out the REV Blinkin firmware has an undocumented 'feature'
  // that means that whenever a specific pulse length is randomly generated,
  // the pulse has a chance of changing the selected strip of the Blinkin
  // 2125 μs for 5v, 2145 μs for 12v
  // Also check out: https://www.chiefdelphi.com/t/rev-blinkin-resetting-strip-mode-randomly/432510/12
  public void setTo5V() {
    try {
      m_pwm.setPulseTimeMicroseconds(2125);
      TimeUnit.MICROSECONDS.sleep(2125*2);// Wait long enough for one pulse to go through
      update();
    } catch (InterruptedException e) {}
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

    // if(!LEDController.isAlive())
    //   status.addReport(ReportLevel.ERROR, "LED is DISCONNECTED");
    
    status.addReport(ReportLevel.INFO, "LED Mode: " + mode.name());

    return status;
  }
  

}