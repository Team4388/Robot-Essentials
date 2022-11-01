// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;

public class SwerveModule extends SubsystemBase {
    public WPI_TalonFX angleMotor;
    public WPI_TalonFX driveMotor;
    private CANCoder canCoder;
    public static Gains m_swerveGains = SwerveDriveConstants.SWERVE_GAINS;

    private static double kEncoderTicksPerRotation = 4096;
    private SwerveModuleState state;
    private double canCoderFeedbackCoefficient;

    public long m_currentTime;
    public long m_lastTime;
    public double m_deltaTime;

    public double m_currentPos;
    public double m_lastPos;

    public SwerveModuleState lastState = new SwerveModuleState();
    public SwerveModuleState currentState;

    /** Creates a new SwerveModule. */
    public SwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX angleMotor, CANCoder canCoder, double offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.canCoder = canCoder;
        canCoderFeedbackCoefficient = canCoder.configGetFeedbackCoefficient();

        TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

        angleTalonFXConfiguration.slot0.kP = m_swerveGains.kP;
        angleTalonFXConfiguration.slot0.kI = m_swerveGains.kI;
        angleTalonFXConfiguration.slot0.kD = m_swerveGains.kD;

        // Use the CANCoder as the remote sensor for the primary TalonFX PID
        angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
        angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
        angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        angleMotor.configAllSettings(angleTalonFXConfiguration);
        // angleMotor.setInverted(true);
        // TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
        // driveTalonFXConfiguration.slot0.kP = 0.05;
        // driveTalonFXConfiguration.slot0.kI = 0.0;
        // driveTalonFXConfiguration.slot0.kD = 0.0;
        // driveTalonFXConfiguration.primaryPID.selectedFeedbackSensor =
        // FeedbackDevice.IntegratedSensor;
        driveMotor.configFactoryDefault();
        driveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        driveMotor.configNominalOutputForward(0, 30);
        driveMotor.configNominalOutputReverse(0, 30);
        driveMotor.configPeakOutputForward(1, 30);
        driveMotor.configPeakOutputReverse(-1, 30);
        driveMotor.configAllowableClosedloopError(0, 0, 30);
        // driveMotor.setInverted(true);
        driveMotor.config_kP(0, 0, 30);
        driveMotor.config_kI(0, 0, 30);
        driveMotor.config_kD(0, 0, 30);
        
        // driveMotor.configAllSettings(driveTalonFXConfiguration);

        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.sensorCoefficient = 0.087890625;
        canCoderConfiguration.magnetOffsetDegrees = offset;
        canCoderConfiguration.sensorDirection = true;
        canCoder.configAllSettings(canCoderConfiguration);

        m_currentTime = System.currentTimeMillis();
        m_lastTime = System.currentTimeMillis();

        m_lastPos = driveMotor.getSelectedSensorPosition();
    }

    private Rotation2d getAngle() {
        // ! Note: This assumes the CANCoders are setup with the default feedback coefficient and the sensor value reports degrees.
        return Rotation2d.fromDegrees(canCoder.getAbsolutePosition());
    }

    /**
     * Set the speed + rotation of the swerve module from a SwerveModuleState object
     *
     * @param desiredState - A SwerveModuleState representing the desired new state
     *                     of the module
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean ignoreAngle) {
        Rotation2d currentRotation = getAngle();
        // currentRotation.getDegrees());
        state = SwerveModuleState.optimize(desiredState, currentRotation);

        // Find the difference between our current rotational position + our new
        // rotational position
        Rotation2d rotationDelta = state.angle.minus(currentRotation);

        // Find the new absolute position of the module based on the difference in
        // rotation
        double deltaTicks = (rotationDelta.getDegrees() / 360.) * kEncoderTicksPerRotation;
        // Convert the CANCoder from it's position reading back to ticks
        double currentTicks = canCoder.getPosition() / canCoderFeedbackCoefficient;
        double desiredTicks = currentTicks + deltaTicks;

        if (!ignoreAngle) {
            angleMotor.set(TalonFXControlMode.Position, desiredTicks);
        }

        // Please work
        double ftPerSec = Units.metersToFeet(state.speedMetersPerSecond);
        double normFtPerSec = ftPerSec / SwerveDriveConstants.MAX_SPEED_FEET_PER_SEC;
        // double angleCorrection = angleMotor.getSelectedSensorVelocity() * 2.69;
        driveMotor.set(normFtPerSec);// - angleMotor.get());
        // driveMotor.set(TalonFXControlMode.Velocity, angleCorrection); // Ratio
        // between axis = 1/1.75 Ratio of wheel is 5.14/1 ratio of steer is 12.8/1
    }

    /**
     * Get current module state.
     *
     * @return The current state of the module in m/s.
     */
    public SwerveModuleState getState() {
        // return state;
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity() * SwerveDriveConstants.INCHES_PER_TICK
                * SwerveDriveConstants.METERS_PER_INCH * 10, getAngle());
    }

    /**
     * Stop the drive and steer motors of current module.
     */
    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public void rotateToAngle(double angle) {
        this.angleMotor.set(TalonFXControlMode.Position, angle);
    }

    @Override
    public void periodic() {
        currentState = this.getState();

        Rotation2d currentRotation = getAngle();
        SmartDashboard.putNumber("Angle Motor " + angleMotor.getDeviceID(), currentRotation.getDegrees());
        SmartDashboard.putNumber("Drive Motor " + driveMotor.getDeviceID(),
                ((driveMotor.getSelectedSensorPosition() / 2048) * 360) % 360);

        lastState = currentState;
    }

    public void reset() {
        canCoder.setPositionToAbsolute();
        // canCoder.configSensorInitializationStrategy(initializationStrategy)
    }
    public double getCurrent(){
        return angleMotor.getSupplyCurrent() + driveMotor.getSupplyCurrent();
    }

    public double getVoltage(){
        return (Math.abs(angleMotor.getMotorOutputVoltage()) + Math.abs(driveMotor.getMotorOutputVoltage()));
    }
}