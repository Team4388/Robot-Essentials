/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.utility;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;
import org.junit.*;

import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.sensors.PigeonIMU;
import frc4388.utility.RobotGyro;

/**
 * Add your docs here.
 */
public class RobotGyroUtilityTest {
    // Arrange
    RobotGyro gyro = new RobotGyro(mock(PigeonIMU.class));

    @Test
    public void testHeading() {
        // Act & Assert
        assertEquals(-90, gyro.getHeading(270), 0.0001);
        assertEquals(-45, gyro.getHeading(315), 0.0001);
        assertEquals(-60, gyro.getHeading(-60), 0.0001);
        assertEquals(30, gyro.getHeading(30), 0.0001);
        assertEquals(0, gyro.getHeading(0), 0.0001);
        assertEquals(180, gyro.getHeading(180), 0.0001);
        assertEquals(-180, gyro.getHeading(-180), 0.0001);
        assertEquals(97, gyro.getHeading(1537), 0.0001);
        assertEquals(99, gyro.getHeading(-2781), 0.0001);
    }
}
