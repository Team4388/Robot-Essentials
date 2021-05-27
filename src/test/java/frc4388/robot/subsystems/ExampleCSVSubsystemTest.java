/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot.subsystems;

import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

/**
 * Add your docs here.
 */
public class ExampleCSVSubsystemTest {

    ExampleCSVSubsystem exampleCSVSubsystem;

    @Before
    public void testConstructor() {
        exampleCSVSubsystem = new ExampleCSVSubsystem();
    }

    @Test
    public void testGetData() {
        assertTrue(exampleCSVSubsystem.m_distanceCSV.getData(true).length > 0);
    }

}
