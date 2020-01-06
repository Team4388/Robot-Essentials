/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc4388.robot.subsystems.Drive;
import frc4388.utility.controller.IHandController;

public class DriveWithJoystick extends CommandBase {

  private final Drive m_drive;
  private final IHandController m_driverXbox;
  public double m_inputMove, m_inputSteer;

  /**
   * Creates a new DriveWithJoystick, driving the robot with the given controller
   */
  public DriveWithJoystick(Drive subsystem, IHandController controller) {
    m_drive = subsystem;
    m_driverXbox = controller;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_inputMove = m_driverXbox.getLeftYAxis();
    m_inputSteer = m_driverXbox.getRightXAxis();
    m_drive.driveWithInput(m_inputMove, m_inputSteer);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
