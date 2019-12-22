/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.Drive;

import edu.wpi.first.wpilibj.command.Command;
import frc4388.robot.OI;
import frc4388.robot.Robot;

public class DriveWithJoystick extends Command {

  public double m_inputMove, m_inputSteer;

  public DriveWithJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_Drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_inputMove = OI.getInstance().getDriverController().getLeftYAxis();
    m_inputSteer = -(OI.getInstance().getDriverController().getRightXAxis());
    Robot.m_Drive.driveWithInput(m_inputMove, m_inputSteer);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
