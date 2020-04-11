/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.utility;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

import com.kauailabs.navx.frc.AHRS;

import org.junit.*;

import frc4388.mocks.MockPigeonIMU;
import frc4388.robot.Constants.DriveConstants;

/**
 * Based on the RobotGyroUtilityTest class
 */
public class UtilityTest {
    private RobotGyro gyroPigeon;
    private RobotGyro gyroNavX;

    @Test
    public void testConstructor() {
        // Arrange
        MockPigeonIMU pigeon = new MockPigeonIMU(DriveConstants.DRIVE_PIGEON_ID);
        AHRS navX = mock(AHRS.class);
        gyroPigeon = new RobotGyro(pigeon);
        gyroNavX = new RobotGyro(navX);

        // Assert
        assertEquals(true, gyroPigeon.m_isGyroAPigeon);
        assertEquals(pigeon, gyroPigeon.getPigeon());
        assertEquals(null, gyroPigeon.getNavX());
        assertEquals(false, gyroNavX.m_isGyroAPigeon);
        assertEquals(navX, gyroNavX.getNavX());
        assertEquals(null, gyroNavX.getPigeon());
    }

    @Test
    public void testHeadingPigeon() {
        // Arrange
        MockPigeonIMU pigeon = new MockPigeonIMU(DriveConstants.DRIVE_PIGEON_ID);
        gyroPigeon = new RobotGyro(pigeon);

        // Act & Assert
        assertEquals(-90, gyroPigeon.getHeading(270), 0.0001);
        assertEquals(-45, gyroPigeon.getHeading(315), 0.0001);
        assertEquals(-60, gyroPigeon.getHeading(-60), 0.0001);
        assertEquals(30, gyroPigeon.getHeading(30), 0.0001);
        assertEquals(0, gyroPigeon.getHeading(0), 0.0001);
        assertEquals(180, gyroPigeon.getHeading(180), 0.0001);
        assertEquals(-180, gyroPigeon.getHeading(-180), 0.0001);
        assertEquals(97, gyroPigeon.getHeading(1537), 0.0001);
        assertEquals(99, gyroPigeon.getHeading(-2781), 0.0001);
    }
}
