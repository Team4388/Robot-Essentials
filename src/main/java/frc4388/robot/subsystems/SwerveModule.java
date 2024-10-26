// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc4388.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.Constants.SwerveDriveConstants;
import frc4388.utility.Gains;

public class SwerveModule extends SubsystemBase {
    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private CANcoder encoder;
    // private final StatusSignal<Double> cc_pos;
    // private final StatusSignal<Double> cc_vel;
    // private int selfid;
    // private ConfigurableDouble offsetGetter;
    private static int swerveId = 0;
    public static Gains swerveGains = SwerveDriveConstants.PIDConstants.SWERVE_GAINS;
  
    
    /** Creates a new SwerveModule. */
    public SwerveModule(TalonFX driveMotor, TalonFX angleMotor, CANcoder encoder, double offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.encoder = encoder;

        var motorCfg = new TalonFXConfiguration()
            .withOpenLoopRamps(
                new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE)
            ).withClosedLoopRamps(
                new ClosedLoopRampsConfigs()
                    .withDutyCycleClosedLoopRampPeriod(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withDutyCycleNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND)
            ).withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(100)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(100)
                    .withSupplyCurrentLimitEnable(true)
            );

        driveMotor.getConfigurator().apply(motorCfg);
        
        TalonFXConfiguration angleConfig = new TalonFXConfiguration()
            .withOpenLoopRamps(
                new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE)
            ).withClosedLoopRamps(
                new ClosedLoopRampsConfigs()
                    .withDutyCycleClosedLoopRampPeriod(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withDutyCycleNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND)
            );

        angleConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        angleConfig.Slot0.kP = swerveGains.kP;
        angleConfig.Slot0.kI = swerveGains.kI;   
        angleConfig.Slot0.kD = swerveGains.kD;

        angleConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        angleMotor.getConfigurator().apply(angleConfig);

        CANcoderConfiguration canconfig = new CANcoderConfiguration();
        canconfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canconfig.MagnetSensor.MagnetOffset = offset;
        encoder.getConfigurator().apply(canconfig);

        rotateToAngle(0);
    }

    // public void go(double ang){
    //     // double curang = this.encoder.getAbsolutePosition().getValue();
    //     System.out.println(getAngle().getDegrees());
    //     rotateToAngle(ang);
    // }
    
    @Override
    public void periodic() {
        //encoder.configMagnetOffset(offsetGetter.get());
        //SmartDashboard.putString("Error Code: " + selfid, getstuff());
        // SmartDashboard.putNumber("Angular Position: " + selfid, getAngle().getDegrees());
        // SmartDashboard.putNumber("Angular Velocity: " + selfid, getAngularVel());
        // SmartDashboard.putNumber("Drive Position: " + selfid, getDrivePos());
        // SmartDashboard.putNumber("Drive Velocity: " + selfid, getDriveVel());
    }
    /**
     * Get the drive motor of the SwerveModule
     * @return the drive motor of the SwerveModule
     */
    public TalonFX getDriveMotor() {
        return this.driveMotor;
    }

    /**
     * Get the angle motor of the SwerveModule
     * @return the angle motor of the SwerveModule
     */
    public TalonFX getAngleMotor() {
        return this.angleMotor;
    }

    /**
     * Get the CANcoder of the SwerveModule
     * @return the CANcoder of the SwerveModule
     */
    public CANcoder getEncoder() {
        return this.encoder;
    }

    /**
     * Get the angle of a SwerveModule as a Rotation2d
     * @return the angle of a SwerveModule as a Rotation2d
     */
    public Rotation2d getAngle() {
        // * Note: This assumes that the CANCoders are setup with the default feedback coefficient and the sensor value reports degrees.
        // return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
        return Rotation2d.fromRotations(encoder.getPosition().getValue());
    }
    
    public double getAngularVel() {
        // return this.angleMotor.getSelectedSensorVelocity();
        return angleMotor.getVelocity().getValueAsDouble();
    }

    public double getDrivePos() {
        // return this.driveMotor.getSelectedSensorPosition() / SwerveDriveConstants.Conversions.TICKS_PER_MOTOR_REV;
        return driveMotor.getPosition().getValueAsDouble();
    }

    public double getDriveVel() {
        // return this.driveMotor.getSelectedSensorVelocity(0);
        return driveMotor.getVelocity().getValueAsDouble();
    }

    public void stop() {
        driveMotor.set(0);
        angleMotor.set(0);
    }

    public void rotateToAngle(double angle) {
        final PositionVoltage m_request = new PositionVoltage(angle);
        angleMotor.setControl(m_request);
    }

    /**
     * Get state of swerve module
     * @return speed in m/s and angle in degrees
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Units.inchesToMeters(driveMotor.getVelocity().getValue() * 
            SwerveDriveConstants.Conversions.INCHES_PER_WHEEL_REV * 
            SwerveDriveConstants.Conversions.WHEEL_REV_PER_MOTOR_REV), 
            getAngle()
        );
    }

    // private SwerveModuleState optimizeState(SwerveModuleState desiredState) {
    //     Rotation2d curRot = this.getAngle();
        
    // }

    /**
     * Returns the current position of the SwerveModule
     * @return The current position of the SwerveModule in meters traveled by the driveMotor and the angle of the angleMotor.
    //  */
    // public SwerveModulePosition getPosition() {
    //     return new SwerveModulePosition(Units.inchesToMeters(driveMotor.getSelectedSensorPosition() * SwerveDriveConstants.Conversions.INCHES_PER_TICK), getAngle());
    // }

    /**
     * Set the speed and rotation of the SwerveModule from a SwerveModuleState object
     * @param desiredState a SwerveModuleState representing the desired new state of the module
    //  */
    public void setDesiredState(SwerveModuleState desiredState) {
        Rotation2d currentRotation = this.getAngle();

        SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);

        // calculate the difference between our current rotational position and our new rotational position
        Rotation2d rotationDelta = state.angle.minus(currentRotation);

        double speed = Units.metersToFeet(state.speedMetersPerSecond) / SwerveDriveConstants.MAX_SPEED_FEET_PER_SECOND;

        rotateToAngle(rotationDelta.getRotations() + currentRotation.getRotations());

        driveMotor.set(Math.max(Math.min(speed, 1.), -1.));
    }

    public void reset() {
        // encoder.setPosition(0);
    }

    // public double getCurrent() {
    //     return angleMotor.getSupplyCurrent() + driveMotor.getSupplyCurrent();
    // }

    // public double getVoltage() {
    //     return (Math.abs(angleMotor.getMotorOutputVoltage()) + Math.abs(driveMotor.getMotorOutputVoltage()));
    // }

    // public String getstuff() {
    //     encoder.getPosition();
    //     return "" + encoder.getLastError().value;
    // }
}
