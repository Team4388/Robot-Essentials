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
    private static long m_currTime = System.currentTimeMillis();
    public static long m_deltaTime = 0;

    private static long m_startRobotTime = m_currTime;
    public static long m_robotTime = 0;
    public static long m_lastRobotTime = 0;

    private static long m_startMatchTime = 0;
    public static long m_matchTime = 0;
    public static long m_lastMatchTime = 0;

    public static long m_frameNumber = 0;

    /**
     * This class should not be instantiated.
     */
    private RobotTime(){}

    /**
     * Call this once per periodic loop.
     */
    public static void updateTimes() {
        m_lastRobotTime = m_robotTime;
        m_lastMatchTime = m_matchTime;

        m_currTime = System.currentTimeMillis();
        m_robotTime = m_currTime - m_startRobotTime;
        m_deltaTime = m_robotTime - m_lastRobotTime;
        m_frameNumber++;

        if (m_matchTime != 0) {
            m_matchTime = m_currTime - m_startMatchTime;
        }
    }

    /**
     * Call this in both the auto and periodic inits
     */
    public static void startMatchTime() {
        if (m_matchTime == 0) {
            m_startMatchTime = m_currTime;
            m_matchTime = 1;
        }
    }

    /**
     * Call this in disabled init
     */
    public static void endMatchTime() {
        m_startMatchTime = 0;
        m_matchTime = 0;
    }
}
