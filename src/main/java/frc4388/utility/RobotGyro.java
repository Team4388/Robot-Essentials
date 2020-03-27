/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.utility;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * Gyro class that allows for interchangeable use between a pigeon and a navX
 */
public class RobotGyro extends GyroBase {
    private PigeonIMU m_pigeon;
    private AHRS m_navX;
    public boolean isGyroAPigeon; //true if pigeon, false if navX

    /**
     * Creates a Gyro based on a pigeon
     * @param gyro the gyroscope to use for Gyro
     */
    public RobotGyro(PigeonIMU gyro) {
        m_pigeon = gyro;
        isGyroAPigeon = true;
    }

    /**
     * Creates a Gyro based on a navX
     * @param gyro the gyroscope to use for Gyro
     */
    public RobotGyro(AHRS gyro){
        m_navX = gyro;
        isGyroAPigeon = false;
    }

    /**
     * <p>NavX:
     * <p>Calibrate the gyro by running for a number of samples and computing the center value. Then use
     * the center value as the Accumulator center value for subsequent measurements. It's important to
     * make sure that the robot is not moving while the centering calculations are in progress, this
     * is typically done when the robot is first turned on while it's sitting at rest before the
     * competition starts.
     * 
     * <p>Pigeon:
     * <p>Calibrate the gyro by collecting data at a range of tempuratures. Allow pigeon to cool, then boot
     * into calibration mode. For faster calibration, use a heat lamp to heat up the pigeon. Once the pigeon
     * has seen a reasonable range of tempuratures, it will exit calibration mode. It's important to
     * make sure that the robot is not moving while the tempurature calculations are in progress, this
     * is typically done when the robot is first turned on while it's sitting at rest before the
     * competition starts.
     */
    @Override
    public void calibrate() {
        if (isGyroAPigeon) {
            m_pigeon.enterCalibrationMode(CalibrationMode.Temperature);
        } else {
            m_navX.calibrate();
        }
    }

    @Override
    public void reset() {
        if (isGyroAPigeon) {
            m_pigeon.setYaw(0);
        } else {
            m_navX.reset();
        }
    }

    /**
	 * Get Yaw, Pitch, and Roll data.
	 *
	 * @return ypr_deg Array with yaw[0], pitch[1], and roll[2] data.
	 *					Yaw is within [-368,640, +368,640] degrees.
	 *					Pitch is within [-90,+90] degrees.
	 *					Roll is within [-90,+90] degrees.
     */
    public double[] getPigeonAngles() {
        double[] angles = new double[3];
        m_pigeon.getYawPitchRoll(angles);
        return angles;
    }

    @Override
    public double getAngle() {
        if (isGyroAPigeon) {
            return getPigeonAngles()[0];
        } else {
            return m_navX.getAngle();
        }
    }

    /**
     * Gets an absolute heading of the robot
     * @return heading from -180 to 180 degrees
     */
    public double getHeading() {
        return Math.IEEEremainder(getAngle(), 360);
    }

    /**
     * Returns the current pitch value (in degrees, from -90 to 90)
     * reported by the sensor.  Pitch is a measure of rotation around
     * the Y Axis.
     * @return The current pitch value in degrees (-90 to 90).
     */
    public double getPitch() {
        if (isGyroAPigeon) {
            return MathUtil.clamp(getPigeonAngles()[1], -90, 90);
        } else {
            return MathUtil.clamp(m_navX.getPitch(), -90, 90);
        }
    }

    /**
     * Returns the current roll value (in degrees, from -90 to 90)
     * reported by the sensor.  Roll is a measure of rotation around
     * the X Axis.
     * @return The current roll value in degrees (-90 to 90).
     */
    public double getRoll() {
        if (isGyroAPigeon) {
            return MathUtil.clamp(getPigeonAngles()[2], -90, 90);
        } else {
            return MathUtil.clamp(m_navX.getRoll(), -90, 90);
        }
    }

    @Override
    public double getRate() {
        if (isGyroAPigeon) {
            return 0;
        } else {
            return m_navX.getRate();
        }
    }

    public PigeonIMU getPigeon(){
        return m_pigeon;
    }

    public AHRS getNavX(){
        return m_navX;
    }

    @Override
    public void close() throws Exception {

    }
}
