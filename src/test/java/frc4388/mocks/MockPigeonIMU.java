/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.mocks;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU;

/**
 * Add your docs here.
 */
public class MockPigeonIMU extends PigeonIMU {
    public int m_deviceNumber;
    public double currentYaw;
    public double currentPitch;
    public double currentRoll;

    public MockPigeonIMU(int deviceNumber) {
        super(deviceNumber);
        m_deviceNumber = deviceNumber;
    }

    @Override
    public ErrorCode setYaw(double angleDeg) {
        currentYaw = angleDeg;
        return ErrorCode.OK;
    }

    /**
     * @param currentPitch the Pitch to set
     */
    public void setCurrentPitch(double currentPitch) {
        this.currentPitch = currentPitch;
    }

    /**
     * @param currentRoll the Roll to set
     */
    public void setCurrentRoll(double currentRoll) {
        this.currentRoll = currentRoll;
    }

    @Override
    public ErrorCode getYawPitchRoll(double[] ypr_deg) {
        ypr_deg[0] = currentYaw;
        ypr_deg[1] = currentPitch;
        ypr_deg[2] = currentRoll;
        return ErrorCode.OK;
    }
}
