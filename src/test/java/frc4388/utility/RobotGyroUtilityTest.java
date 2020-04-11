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
import com.kauailabs.navx.frc.AHRS;

import frc4388.mocks.MockPigeonIMU;
import frc4388.robot.Constants.DriveConstants;
import frc4388.utility.RobotGyro;

/**
 * Add your docs here.
 */
public class RobotGyroUtilityTest {
    MockPigeonIMU pigeon = new MockPigeonIMU(DriveConstants.DRIVE_PIGEON_ID);
    RobotGyro gyroPigeon = new RobotGyro(pigeon);
    AHRS navX = mock(AHRS.class);
    RobotGyro gyroNavX = new RobotGyro(navX);
    RobotTime robotTime = RobotTime.getInstance();

    // TODO UNTESTED: most functions for NavX

    @Test
    public void testConfig() {
        // TEST 1
        assertEquals(true, gyroPigeon.m_isGyroAPigeon);
        assertEquals(pigeon, gyroPigeon.getPigeon());
        assertEquals(null, gyroPigeon.getNavX());

        // TEST 2
        assertEquals(false, gyroNavX.m_isGyroAPigeon);
        assertEquals(navX, gyroNavX.getNavX());
        assertEquals(null, gyroNavX.getPigeon());
    }

    @Test
    public void testHeading() {
        // TESTS
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
    public void testYawPitchRoll() {        
        // TEST 1
        assertEquals(0, gyroPigeon.getAngle(), 0.0001);

        // TEST 2
        pigeon.setYaw(40);
        assertEquals(40, gyroPigeon.getAngle(), 0.0001);

        // TEST 3
        gyroPigeon.reset();
        assertEquals(0, gyroPigeon.getAngle(), 0.0001);

        // TEST 4
        pigeon.setYaw(-1457);
        pigeon.setCurrentPitch(100);
        pigeon.setCurrentRoll(100);
        assertEquals(-1457, gyroPigeon.getAngle(), 0.0001);
        assertEquals(90, gyroPigeon.getPitch(), 0.0001);
        assertEquals(90, gyroPigeon.getRoll(), 0.0001);

        // TEST 5
        pigeon.setCurrentPitch(45);
        pigeon.setCurrentRoll(45);
        assertEquals(45, gyroPigeon.getPitch(), 0.0001);
        assertEquals(45, gyroPigeon.getRoll(), 0.0001);

        // TEST 6
        pigeon.setCurrentPitch(0);
        pigeon.setCurrentRoll(0);
        assertEquals(0, gyroPigeon.getPitch(), 0.0001);
        assertEquals(0, gyroPigeon.getRoll(), 0.0001);

        // TEST 7
        pigeon.setCurrentPitch(-60);
        pigeon.setCurrentRoll(-60);
        assertEquals(-60, gyroPigeon.getPitch(), 0.0001);
        assertEquals(-60, gyroPigeon.getRoll(), 0.0001);

        // TEST 8
        pigeon.setCurrentPitch(-90);
        pigeon.setCurrentRoll(-90);
        assertEquals(-90, gyroPigeon.getPitch(), 0.0001);
        assertEquals(-90, gyroPigeon.getRoll(), 0.0001);

        // TEST 9
        pigeon.setCurrentPitch(-100);
        pigeon.setCurrentRoll(-100);
        assertEquals(-90, gyroPigeon.getPitch(), 0.0001);
        assertEquals(-90, gyroPigeon.getRoll(), 0.0001);
    }

    @Test
    public void testRates() {
        // SETUP
        pigeon.setYaw(0);
        gyroPigeon.updatePigeonDeltas();

        // TEST 1
        robotTime.m_deltaTime = 5;
        pigeon.setYaw(0);
        gyroPigeon.updatePigeonDeltas();
        assertEquals(0, gyroPigeon.getRate(), 1);

        // TEST 2
        robotTime.m_deltaTime = 5;
        pigeon.setYaw(90);
        gyroPigeon.updatePigeonDeltas();
        assertEquals(18000, gyroPigeon.getRate(), 0.001);

        // TEST 3
        robotTime.m_deltaTime = 5;
        pigeon.setYaw(90);
        gyroPigeon.updatePigeonDeltas();
        assertEquals(0, gyroPigeon.getRate(), 0.001);

        // TEST 4
        robotTime.m_deltaTime = 3;
        pigeon.setYaw(-30);
        gyroPigeon.updatePigeonDeltas();
        assertEquals(-40000, gyroPigeon.getRate(), 0.001);

        // TEST 5
        robotTime.m_deltaTime = 6;
        pigeon.setYaw(690);
        gyroPigeon.updatePigeonDeltas();
        assertEquals(120000, gyroPigeon.getRate(), 0.001);
    }

    private void wait(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {}
    }
}
