/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;
import org.junit.*;
import edu.wpi.first.wpilibj.*;
import frc4388.robot.Constants.LEDConstants;
import frc4388.utility.LEDPatterns;

/**
 * Add your docs here.
 */
public class LEDSubsystemTest {
    Spark ledController = mock(Spark.class);
    LED led = new LED(ledController);

    @Test
    public void testPatterns() {
        assertEquals(LEDConstants.DEFAULT_PATTERN.getValue(), led.getPattern().getValue(), 0.0001);

        led.setPattern(LEDPatterns.RAINBOW_RAINBOW);
        assertEquals(LEDPatterns.RAINBOW_RAINBOW.getValue(), led.getPattern().getValue(), 0.0001);

        led.setPattern(LEDPatterns.BLUE_BREATH);
        assertEquals(LEDPatterns.BLUE_BREATH.getValue(), led.getPattern().getValue(), 0.0001);

        led.setPattern(LEDPatterns.C1_SCANNER);
        assertEquals(LEDPatterns.C1_SCANNER.getValue(), led.getPattern().getValue(), 0.0001);

        led.setPattern(LEDPatterns.C1_CHASE);
        assertEquals(LEDPatterns.C1_CHASE.getValue(), led.getPattern().getValue(), 0.0001);

        led.setPattern(LEDPatterns.SOLID_BLACK);
        assertEquals(LEDPatterns.SOLID_BLACK.getValue(), led.getPattern().getValue(), 0.0001);
    }
}
