package frc4388.robot.commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

/**
 * A command composition that runs one of two commands, depending on the value of the given
 * condition when this command is initialized.
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually, and the composition requires all
 * subsystems its components require.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class WhileTrueCommand extends Command {
  private final Command m_whileTrue;
  private final BooleanSupplier m_condition;

  /**
   * Creates a new WhileTrueCommand.
   *
   * @param whileTrue the command to run while the condition is true
   * @param condition the condition to determine which command to run
   */
  @SuppressWarnings("this-escape")
  public WhileTrueCommand(Command whileTrue, BooleanSupplier condition) {
    m_whileTrue = requireNonNullParam(whileTrue, "whileTrue", "WhileTrueCommand");
    m_condition = requireNonNullParam(condition, "condition", "WhileTrueCommand");

    //CommandScheduler.getInstance().registerComposedCommands(whileTrue);

    // addRequirements(whileTrue.getRequirements());
  }

  @Override
  public void initialize() {
    if(m_condition.getAsBoolean())
      m_whileTrue.initialize();
  }

  @Override
  public void execute() {
    m_whileTrue.execute();

    System.out.println("Loop, " + !m_whileTrue.isFinished() + ", " +  m_condition.getAsBoolean());

    if(!m_whileTrue.isFinished())
      return;

    if(m_condition.getAsBoolean()){
      m_whileTrue.end(false);
      m_whileTrue.initialize();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_whileTrue.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return !m_condition.getAsBoolean() && m_whileTrue.isFinished();
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_whileTrue.runsWhenDisabled();
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    if (m_whileTrue.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
      return InterruptionBehavior.kCancelSelf;
    } else {
      return InterruptionBehavior.kCancelIncoming;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addStringProperty("whileTrue", m_whileTrue::getName, null);
    builder.addStringProperty(
        "selected",
        () -> {
          if (m_whileTrue == null) {
            return "null";
          } else {
            return m_whileTrue.getName();
          }
        },
        null);
  }
}
