/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc4388.robot.subsystems.*;
import org.junit.*;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

public class CommandTest {
    private CommandScheduler scheduler = null;

    @Before
    public void setup() {
        scheduler = CommandScheduler.getInstance();
    }

    // TODO: Update this to use an actual command. Won't work with inline commands for some reason

    @Test
    public void testExample() {
        // Arrange
        Drive drive = mock(Drive.class);
        RunCommand command = new RunCommand(() -> drive.driveWithInput(0, 0), drive);

        // Act
        scheduler.schedule(command);
        scheduler.run();

        // Assert
        verify(drive).driveWithInput(0, 0);
    }
}
