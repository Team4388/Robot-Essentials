/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.utility.compute;

/**
 * <p>Keeps track of Robot times like time passed, delta time, etc
 * <p>All times are in milliseconds
 */
public class RobotTime {
    private long m_currTime = System.currentTimeMillis();
    public long m_deltaTime = 0;

    private long m_startRobotTime = m_currTime;
    public long m_robotTime = 0;
    public long m_lastRobotTime = 0;

    private long m_startMatchTime = 0;
    public  long m_matchTime = 0;
    public long m_lastMatchTime = 0;

    public long m_frameNumber = 0;

    /**
     * Private constructor prevents other classes from instantiating
     */
    private RobotTime(){}

    private static RobotTime instance = null;

    /**
     * Gets the instance of Robot Time. If there is no instance running one will be created.
     * @return instance of Robot Time
     */
    public static RobotTime getInstance() {
        if (instance == null) {
            instance = new RobotTime();
        }
        return instance;
    }

    /**
     * Call this once per periodic loop.
     */
    public void updateTimes() {
        m_lastRobotTime = m_robotTime;
        m_lastMatchTime = m_matchTime;

        m_currTime = System.currentTimeMillis();
        m_robotTime = m_currTime - m_startRobotTime;
        m_deltaTime = m_robotTime - m_lastRobotTime;
        m_frameNumber++;

        if (m_startMatchTime != 0) {
            m_matchTime = m_currTime - m_startMatchTime;
        }
    }

    /**
     * Call this in both the auto and periodic inits
     */
    public void startMatchTime() {
        if (m_startMatchTime == 0) {
            m_startMatchTime = m_currTime;
        }
    }

    /**
     * Call this in disabled init
     */
    public void endMatchTime() {
        m_startMatchTime = 0;
        m_matchTime = 0;
    }
}
