/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.utility;

/**
 * <p>Keeps track of Robot times like time passed, delta time, etc
 * <p>All times are in milliseconds
 */
public final class RobotTime {
    private static long currTime = System.currentTimeMillis();
    public static long deltaTime = 0;

    private static long startRobotTime = currTime;
    public static long robotTime = 0;
    public static long lastRobotTime = 0;

    private static long startMatchTime = 0;
    public static long matchTime = 0;
    public static long lastMatchTime = 0;

    public static long frameNumber = 0;

    /**
     * This class should not be instantiated.
     */
    private RobotTime(){}

    /**
     * Call this once per periodic loop.
     */
    public static void updateTimes() {
        lastRobotTime = robotTime;
        lastMatchTime = matchTime;

        currTime = System.currentTimeMillis();
        robotTime = currTime - startRobotTime;
        deltaTime = robotTime - lastRobotTime;
        frameNumber++;

        if (matchTime != 0) {
            matchTime = currTime - startMatchTime;
        }
    }

    /**
     * Call this in both the auto and periodic inits
     */
    public static void startMatchTime() {
        if (matchTime == 0) {
            startMatchTime = currTime;
            matchTime = 1;
        }
    }

    /**
     * Call this in disabled init
     */
    public static void endMatchTime() {
        startMatchTime = 0;
        matchTime = 0;
    }
}
