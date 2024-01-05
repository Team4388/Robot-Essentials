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
 * Add your docs here.
 */
public class RobotGyroUtilityTest {
    // TODO UNTESTED: most functions for NavX

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

    @Test
    public void testYawPitchRollPigeon() {
        // Arrange
        MockPigeonIMU pigeon = new MockPigeonIMU(DriveConstants.DRIVE_PIGEON_ID);
        gyroPigeon = new RobotGyro(pigeon);

        // Assert
        assertEquals(0, gyroPigeon.getAngle(), 0.0001);

        // Act
        pigeon.setYaw(40);

        // Assert
        assertEquals(40, gyroPigeon.getAngle(), 0.0001);

        // Act
        gyroPigeon.reset();

        // Assert
        assertEquals(0, gyroPigeon.getAngle(), 0.0001);

        // Act
        pigeon.setYaw(-1457);
        pigeon.setCurrentPitch(100);
        pigeon.setCurrentRoll(100);

        // Assert
        assertEquals(-1457, gyroPigeon.getAngle(), 0.0001);
        assertEquals(90, gyroPigeon.getPitch(), 0.0001);
        assertEquals(90, gyroPigeon.getRoll(), 0.0001);

        // Act
        pigeon.setCurrentPitch(45);
        pigeon.setCurrentRoll(45);

        // Assert
        assertEquals(45, gyroPigeon.getPitch(), 0.0001);
        assertEquals(45, gyroPigeon.getRoll(), 0.0001);

        // Act
        pigeon.setCurrentPitch(0);
        pigeon.setCurrentRoll(0);

        // Assert
        assertEquals(0, gyroPigeon.getPitch(), 0.0001);
        assertEquals(0, gyroPigeon.getRoll(), 0.0001);

        // Act
        pigeon.setCurrentPitch(-60);
        pigeon.setCurrentRoll(-60);

        // Assert
        assertEquals(-60, gyroPigeon.getPitch(), 0.0001);
        assertEquals(-60, gyroPigeon.getRoll(), 0.0001);

        // Act
        pigeon.setCurrentPitch(-90);
        pigeon.setCurrentRoll(-90);

        // Assert
        assertEquals(-90, gyroPigeon.getPitch(), 0.0001);
        assertEquals(-90, gyroPigeon.getRoll(), 0.0001);

        // Act
        pigeon.setCurrentPitch(-100);
        pigeon.setCurrentRoll(-100);

        // Assert
        assertEquals(-90, gyroPigeon.getPitch(), 0.0001);
        assertEquals(-90, gyroPigeon.getRoll(), 0.0001);
    }

    @Test
    public void testRatesPigeon() {
        // Arrange
        MockPigeonIMU pigeon = new MockPigeonIMU(DriveConstants.DRIVE_PIGEON_ID);
        gyroPigeon = new RobotGyro(pigeon);
        RobotTime robotTime = RobotTime.getInstance();
        gyroPigeon.updatePigeonDeltas();

        // Act
        robotTime.m_deltaTime = 5;
        pigeon.setYaw(0);
        gyroPigeon.updatePigeonDeltas();

        // Assert
        assertEquals(0, gyroPigeon.getRate(), 1);

        // Act
        robotTime.m_deltaTime = 5;
        pigeon.setYaw(90);
        gyroPigeon.updatePigeonDeltas();

        // Assert
        assertEquals(18000, gyroPigeon.getRate(), 0.001);

        // Act
        robotTime.m_deltaTime = 5;
        pigeon.setYaw(90);
        gyroPigeon.updatePigeonDeltas();

        // Assert
        assertEquals(0, gyroPigeon.getRate(), 0.001);

        // Act
        robotTime.m_deltaTime = 3;
        pigeon.setYaw(-30);
        gyroPigeon.updatePigeonDeltas();
        
        // Assert
        assertEquals(-40000, gyroPigeon.getRate(), 0.001);

        // Act
        robotTime.m_deltaTime = 6;
        pigeon.setYaw(690);
        gyroPigeon.updatePigeonDeltas();

        // Assert
        assertEquals(120000, gyroPigeon.getRate(), 0.001);
    }
}
