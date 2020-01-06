/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.LED;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc4388.robot.constants.LEDPatterns;
import frc4388.robot.subsystems.LED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetLEDPattern extends InstantCommand {

  private final LED m_led;
  public static LEDPatterns m_pattern;

  public SetLEDPattern(LED subsystem, LEDPatterns pattern) {
    m_led = subsystem;
    m_pattern = pattern;
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.setPattern(m_pattern);
  }
}
