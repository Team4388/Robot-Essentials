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

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

/**
 * Add your docs here.
 */
public class UtilityTest {
    @Test
    public void testExample() {
        // Arrange
        boolean isFalse = false;
        int i = 0;
        double d = 0.0;

        // Act
        wait(1);
        isFalse = !isFalse;
        i++;
        d -= Math.PI;

        // Assert
        assertEquals(1, i);
        assertEquals(-Math.PI, d, 0.001);
        assertEquals(true, isFalse);
    }

    private void wait(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {}
    }
}
