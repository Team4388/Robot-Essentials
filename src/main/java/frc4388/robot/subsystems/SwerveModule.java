// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;

public class SwerveModule extends SubsystemBase {
    private WPI_TalonFX driveMotor;
    private WPI_TalonFX angleMotor;
    private CANCoder encoder;

    public static Gains swerveGains = SwerveDriveConstants.PIDConstants.SWERVE_GAINS;
  
    /** Creates a new SwerveModule. */
    public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, CANCoder encoder, double offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.encoder = encoder;

        TalonFXConfiguration angleConfig = new TalonFXConfiguration();
        angleConfig.slot0.kP = swerveGains.kP;
        angleConfig.slot0.kI = swerveGains.kI;
        angleConfig.slot0.kD = swerveGains.kD;

        // use the CANcoder as the remote sensor for the primary TalonFX PID
        angleConfig.remoteFilter0.remoteSensorDeviceID = encoder.getDeviceID();
        angleConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        angleConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        angleMotor.configAllSettings(angleConfig);

        encoder.configMagnetOffset(offset);
        
        driveMotor.setSelectedSensorPosition(0);
        driveMotor.config_kP(0, 0.2);
    }

    /**
     * Get the drive motor of the SwerveModule
     * @return the drive motor of the SwerveModule
     */
    public WPI_TalonFX getDriveMotor() {
        return this.driveMotor;
    }

    /**
     * Get the angle motor of the SwerveModule
     * @return the angle motor of the SwerveModule
     */
    public WPI_TalonFX getAngleMotor() {
        return this.angleMotor;
    }

    /**
     * Get the CANcoder of the SwerveModule
     * @return the CANcoder of the SwerveModule
     */
    public CANCoder getEncoder() {
        return this.encoder;
    }

    /**
     * Get the angle of a SwerveModule as a Rotation2d
     * @return the angle of a SwerveModule as a Rotation2d
     */
    public Rotation2d getAngle() {
        // * Note: This assumes that the CANCoders are setup with the default feedback coefficient and the sensor value reports degrees.
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
    }
    
    public double getAngularVel() {
        return this.angleMotor.getSelectedSensorVelocity();
    }

    public double getDrivePos() {
        return this.driveMotor.getSelectedSensorPosition() / SwerveDriveConstants.Conversions.TICKS_PER_MOTOR_REV;
    }

    public double getDriveVel() {
        return this.driveMotor.getSelectedSensorVelocity(0);
    }

    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public void rotateToAngle(double angle) {
        angleMotor.set(TalonFXControlMode.Position, angle);
    }

    /**
     * Get state of swerve module
     * @return speed in m/s and angle in degrees
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Units.inchesToMeters(driveMotor.getSelectedSensorVelocity() * SwerveDriveConstants.Conversions.INCHES_PER_TICK) * SwerveDriveConstants.Conversions.TICK_TIME_TO_SECONDS, 
            getAngle()
        );
    }

    /**
     * Returns the current position of the SwerveModule
     * @return The current position of the SwerveModule in meters traveled by the driveMotor and the angle of the angleMotor.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(Units.inchesToMeters(driveMotor.getSelectedSensorPosition() * SwerveDriveConstants.Conversions.INCHES_PER_TICK), getAngle());
    }

    /**
     * Set the speed and rotation of the SwerveModule from a SwerveModuleState object
     * @param desiredState a SwerveModuleState representing the desired new state of the module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = this.getAngle();

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

        // calculate the difference between our current rotational position and our new rotational position
        Rotation2d rotationDelta = state.angle.minus(currentRotation);

        // calculate the new absolute position of the SwerveModule based on the difference in rotation
        double deltaTicks = (rotationDelta.getDegrees() / 360.) * SwerveDriveConstants.Conversions.CANCODER_TICKS_PER_ROTATION;

        // convert the CANCoder from its position reading to ticks
        double currentTicks = encoder.getPosition() / encoder.configGetFeedbackCoefficient();

        angleMotor.set(TalonFXControlMode.Position, currentTicks + deltaTicks);

        double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);

        driveMotor.set((feetPerSecond / SwerveDriveConstants.MAX_SPEED_FEET_PER_SECOND));
    }

    public void reset(double position) {
        encoder.setPositionToAbsolute();
    }

    public double getCurrent() {
        return angleMotor.getSupplyCurrent() + driveMotor.getSupplyCurrent();
    }

    public double getVoltage() {
        return (Math.abs(angleMotor.getMotorOutputVoltage()) + Math.abs(driveMotor.getMotorOutputVoltage()));
    }
}
