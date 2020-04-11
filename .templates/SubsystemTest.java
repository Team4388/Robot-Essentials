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
    public void testConstructor() {
        // Arrange
        Spark ledController = mock(Spark.class);

        // Act
        LED led = new LED(ledController);

        // Assert
        assertEquals(LEDConstants.DEFAULT_PATTERN.getValue(), led.getPattern().getValue(), 0.0001);
    }

    @Test
    public void testPatterns() {
        // Arrange
        Spark ledController = mock(Spark.class);
        LED led = new LED(ledController);

        // Act
        led.setPattern(LEDPatterns.RAINBOW_RAINBOW);

        // Assert
        assertEquals(LEDPatterns.RAINBOW_RAINBOW.getValue(), led.getPattern().getValue(), 0.0001);

        // Act
        led.setPattern(LEDPatterns.BLUE_BREATH);

        // Assert
        assertEquals(LEDPatterns.BLUE_BREATH.getValue(), led.getPattern().getValue(), 0.0001);

        // Act
        led.setPattern(LEDPatterns.SOLID_BLACK);

        // Assert
        assertEquals(LEDPatterns.SOLID_BLACK.getValue(), led.getPattern().getValue(), 0.0001);
    }

    private void wait(int millis) {
        try {
            Thread.sleep(millis);
        } catch (Exception e) {}
    }
}
