/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc4388.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc4388.utility.CanDevice;
import frc4388.utility.Gains;
import frc4388.utility.LEDPatterns;
import frc4388.utility.ReefPositionHelper;
import frc4388.utility.Trim;
import frc4388.utility.configurable.ConfigurableDouble;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final String CANBUS_NAME = "rio";
    
    public static final class SwerveDriveConstants {

        public static final double MAX_ROT_SPEED        = Math.PI * 2;
        public static final double AUTO_MAX_ROT_SPEED = 1.5;
        public static final double MIN_ROT_SPEED        = 1.0;
        public static       double ROTATION_SPEED       = MAX_ROT_SPEED;
        public static       double PLAYBACK_ROTATION_SPEED = AUTO_MAX_ROT_SPEED;
        public static       double ROT_CORRECTION_SPEED = 10; // MIN_ROT_SPEED;

        public static final double CORRECTION_MIN = 10;
        public static final double CORRECTION_MAX = 50;
        
        public static final double SLOW_SPEED = 0.25;
        public static final double FAST_SPEED = 0.5;
        public static final double TURBO_SPEED = 1.0;
        
        public static final double[] GEARS = {SLOW_SPEED, FAST_SPEED, TURBO_SPEED};
        public static final int STARTING_GEAR = 0;
        // Dimensions
        public static final double WIDTH = 18.5; // TODO: validate
        public static final double HEIGHT = 18.5; // TODO: validate
        public static final double HALF_WIDTH = WIDTH / 2.d;
        public static final double HALF_HEIGHT = HEIGHT / 2.d;

        // Mechanics
        private static final double COUPLE_RATIO = 3; //todo: find
        private static final double DRIVE_RATIO = 6.12;
        private static final double STEER_RATIO = (150/7);
        private static final Distance WHEEL_RADIUS = Inches.of(2);
        
        public static final double MAX_SPEED_MEETERS_PER_SEC = 6.22; // TODO: Validate

        public static final double MAX_SPEED_FEET_PER_SECOND = MAX_SPEED_MEETERS_PER_SEC * 3.28084;
        public static final double MAX_ANGULAR_SPEED_FEET_PER_SECOND = 2 * 2 * Math.PI;

        // Operation
        public static final double FORWARD_OFFSET = 90; // 0, 90, 180, 270

        public static final boolean DRIFT_CORRECTION_ENABLED = true;
        public static final boolean INVERT_X = false;
        public static final boolean INVERT_Y = true;
        public static final boolean INVERT_ROTATION = false;

        // public static final Trim POINTLESS_TRIM = new Trim("Pointless Trim", Double.MAX_VALUE, Double.MIN_VALUE, 0.1, 0);

        private static final class ModuleSpecificConstants { //2025
            //Front Left
            private static final Angle FRONT_LEFT_ENCODER_OFFSET = Rotations.of(-0.368896484375);
            private static final boolean FRONT_LEFT_DRIVE_MOTOR_INVERTED = false;
            private static final boolean FRONT_LEFT_STEER_MOTOR_INVERTED = true;
            private static final boolean FRONT_LEFT_ENCODER_INVERTED = false;
            private static final Distance FRONT_LEFT_XPOS = Inches.of(HALF_WIDTH);
            private static final Distance FRONT_LEFT_YPOS = Inches.of(HALF_HEIGHT);
            
            //Front Right
            private static final Angle FRONT_RIGHT_ENCODER_OFFSET = Rotations.of(-0.011474609375);
            private static final boolean FRONT_RIGHT_DRIVE_MOTOR_INVERTED = true;
            private static final boolean FRONT_RIGHT_STEER_MOTOR_INVERTED = true;
            private static final boolean FRONT_RIGHT_ENCODER_INVERTED = false;
            private static final Distance FRONT_RIGHT_XPOS = Inches.of(HALF_WIDTH);
            private static final Distance FRONT_RIGHT_YPOS = Inches.of(-HALF_HEIGHT);

            //Back Left
            private static final Angle BACK_LEFT_ENCODER_OFFSET = Rotations.of(0.333251953125+0.5);
            private static final boolean BACK_LEFT_DRIVE_MOTOR_INVERTED = false;
            private static final boolean BACK_LEFT_STEER_MOTOR_INVERTED = true;
            private static final boolean BACK_LEFT_ENCODER_INVERTED = false;
            private static final Distance BACK_LEFT_XPOS = Inches.of(-HALF_WIDTH);
            private static final Distance BACK_LEFT_YPOS = Inches.of(HALF_HEIGHT);
            
            //Back Right
            private static final Angle BACK_RIGHT_ENCODER_OFFSET = Rotations.of(0.4306640625+0.5);
            private static final boolean BACK_RIGHT_DRIVE_MOTOR_INVERTED = false;
            private static final boolean BACK_RIGHT_STEER_MOTOR_INVERTED = true;
            private static final boolean BACK_RIGHT_ENCODER_INVERTED = false;
            private static final Distance BACK_RIGHT_XPOS = Inches.of(-HALF_WIDTH);
            private static final Distance BACK_RIGHT_YPOS = Inches.of(-HALF_HEIGHT);
        }

        public static final class IDs {
            public static final CanDevice RIGHT_FRONT_WHEEL   = new CanDevice("RIGHT_FRONT_WHEEL", 4);
            public static final CanDevice RIGHT_FRONT_STEER   = new CanDevice("RIGHT_FRONT_STEER", 5);
            public static final CanDevice RIGHT_FRONT_ENCODER = new CanDevice("RIGHT_FRONT_ENCODER", 11);
            
            public static final CanDevice LEFT_FRONT_WHEEL    = new CanDevice("LEFT_FRONT_WHEEL", 2);
            public static final CanDevice LEFT_FRONT_STEER    = new CanDevice("LEFT_FRONT_STEER", 3);
            public static final CanDevice LEFT_FRONT_ENCODER  = new CanDevice("LEFT_FRONT_ENCODER", 10);
            
            public static final CanDevice LEFT_BACK_WHEEL     = new CanDevice("LEFT_BACK_WHEEL", 6);
            public static final CanDevice LEFT_BACK_STEER     = new CanDevice("LEFT_BACK_STEER", 7);
            public static final CanDevice LEFT_BACK_ENCODER   = new CanDevice("LEFT_BACK_ENCODER", 12);
            
            public static final CanDevice RIGHT_BACK_WHEEL    = new CanDevice("RIGHT_BACK_WHEEL", 8);  
            public static final CanDevice RIGHT_BACK_STEER    = new CanDevice("RIGHT_BACK_STEER", 9);
            public static final CanDevice RIGHT_BACK_ENCODER  = new CanDevice("RIGHT_BACK_ENCODER", 13);

            public static final CanDevice DRIVE_PIGEON        = new CanDevice("DRIVE_PIGEON", 14);
        }
    
        public static final class PIDConstants {
            public static final int SWERVE_SLOT_IDX = 0;
            public static final int SWERVE_PID_LOOP_IDX = 1;
            public static final Gains SWERVE_GAINS = new Gains(50, 0.0, 0.32, 0.0, 0, 0.0);

            public static final Slot0Configs CURRENT_SWERVE_ROT_GAINS = new Slot0Configs()
                .withKP(50).withKI(0).withKD(0.32)
                .withKS(0).withKV(0).withKA(0);
            
                public static final Slot0Configs TEST_SWERVE_ROT_GAINS = new Slot0Configs()
                .withKP(10).withKI(0).withKD(0.3)
                .withKS(0).withKV(0).withKA(0);

            public static final Gains TEST_SWERVE_GAINS = new Gains(1.2, 0.0, 0.0, 0.0, 0, 0.0);

            // The steer motor uses any SwerveModule.SteerRequestType control request with the
            // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
            public static final Slot0Configs PREPROVIDED_STEER_GAINS = new Slot0Configs()
                .withKP(100).withKI(0).withKD(0.6)
                .withKS(0.1).withKV(1.91).withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
            // When using closed-loop control, the drive motor uses the control
            // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
            public static final Slot0Configs PREPROVIDED_DRIVE_GAINS = new Slot0Configs()
                .withKP(0.1).withKI(0).withKD(0)
                .withKS(0).withKV(0.124);
            
            public static final Gains DRIFT_CORRECTION_GAINS = new Gains(2.5, 0, 0.1);
            public static final Gains RELATIVE_LOCKED_ANGLE_GAINS = new Gains(10, 0, 1);
        }

        public static final class Configurations {
            public static final double OPEN_LOOP_RAMP_RATE = 0.4; // Todo: Test. think this will help.
            public static final double CLOSED_LOOP_RAMP_RATE = 0.4; // Todo: Test. think this will help.
            public static final double NEUTRAL_DEADBAND = 0.04;

            // POWER! (limiting)
            private static final TalonFXConfiguration DRIVE_MOTOR_CONFIGS = new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withDutyCycleNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND)
                ).withOpenLoopRamps(
                    new OpenLoopRampsConfigs()
                        .withDutyCycleOpenLoopRampPeriod(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE)
                ).withClosedLoopRamps(
                    new ClosedLoopRampsConfigs()
                        .withDutyCycleClosedLoopRampPeriod(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE)
            );
            private static final TalonFXConfiguration STEER_MOTOR_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(40) // TODO: tune???
                        .withStatorCurrentLimitEnable(true)
                    ).withMotorOutput(
                        new MotorOutputConfigs()
                            .withNeutralMode(NeutralModeValue.Brake)
                            .withDutyCycleNeutralDeadband(SwerveDriveConstants.Configurations.NEUTRAL_DEADBAND)
                    // ).withOpenLoopRamps(
                    //     new OpenLoopRampsConfigs()
                    //         .withDutyCycleOpenLoopRampPeriod(SwerveDriveConstants.Configurations.OPEN_LOOP_RAMP_RATE)
                    // ).withClosedLoopRamps(
                    //     new ClosedLoopRampsConfigs()
                    //         .withDutyCycleClosedLoopRampPeriod(SwerveDriveConstants.Configurations.CLOSED_LOOP_RAMP_RATE)
            );
            public static final double SLIP_CURRENT = 60; // TODO: Tune??? 
        }

        // No mans land
        // Beware, there be dragons.
        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(IDs.DRIVE_PIGEON.id);

        private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
            new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>() // holy verbosity batman.
                .withDriveMotorGearRatio(DRIVE_RATIO)
                .withSteerMotorGearRatio(STEER_RATIO)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withWheelRadius(WHEEL_RADIUS)
                .withSteerMotorGains(PIDConstants.PREPROVIDED_STEER_GAINS) // ?
                .withDriveMotorGains(PIDConstants.PREPROVIDED_DRIVE_GAINS) // ?
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSlipCurrent(Configurations.SLIP_CURRENT)
                .withSpeedAt12Volts(MAX_SPEED_MEETERS_PER_SEC)
                .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
                .withDriveMotorInitialConfigs(Configurations.DRIVE_MOTOR_CONFIGS)
                .withSteerMotorInitialConfigs(Configurations.STEER_MOTOR_CONFIGS);

        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT =
            ConstantCreator.createModuleConstants(
                IDs.LEFT_FRONT_STEER.id, IDs.LEFT_FRONT_WHEEL.id, IDs.LEFT_FRONT_ENCODER.id, ModuleSpecificConstants.FRONT_LEFT_ENCODER_OFFSET,
                ModuleSpecificConstants.FRONT_LEFT_XPOS, ModuleSpecificConstants.FRONT_LEFT_YPOS, 
                ModuleSpecificConstants.FRONT_LEFT_DRIVE_MOTOR_INVERTED, ModuleSpecificConstants.FRONT_LEFT_STEER_MOTOR_INVERTED, ModuleSpecificConstants.FRONT_LEFT_ENCODER_INVERTED
        );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT =
            ConstantCreator.createModuleConstants(
                IDs.RIGHT_FRONT_STEER.id, IDs.RIGHT_FRONT_WHEEL.id, IDs.RIGHT_FRONT_ENCODER.id, ModuleSpecificConstants.FRONT_RIGHT_ENCODER_OFFSET,
                ModuleSpecificConstants.FRONT_RIGHT_XPOS, ModuleSpecificConstants.FRONT_RIGHT_YPOS, 
                ModuleSpecificConstants.FRONT_RIGHT_DRIVE_MOTOR_INVERTED, ModuleSpecificConstants.FRONT_RIGHT_STEER_MOTOR_INVERTED, ModuleSpecificConstants.FRONT_RIGHT_ENCODER_INVERTED
        );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT =
            ConstantCreator.createModuleConstants(
                IDs.LEFT_BACK_STEER.id, IDs.LEFT_BACK_WHEEL.id, IDs.LEFT_BACK_ENCODER.id, ModuleSpecificConstants.BACK_LEFT_ENCODER_OFFSET,
                ModuleSpecificConstants.BACK_LEFT_XPOS, ModuleSpecificConstants.BACK_LEFT_YPOS, 
                ModuleSpecificConstants.BACK_LEFT_DRIVE_MOTOR_INVERTED, ModuleSpecificConstants.BACK_LEFT_STEER_MOTOR_INVERTED, ModuleSpecificConstants.BACK_LEFT_ENCODER_INVERTED
        );
        public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT =
            ConstantCreator.createModuleConstants(
                IDs.RIGHT_BACK_STEER.id, IDs.RIGHT_BACK_WHEEL.id, IDs.RIGHT_BACK_ENCODER.id, ModuleSpecificConstants.BACK_RIGHT_ENCODER_OFFSET,
                ModuleSpecificConstants.BACK_RIGHT_XPOS, ModuleSpecificConstants.BACK_RIGHT_YPOS, 
                ModuleSpecificConstants.BACK_RIGHT_DRIVE_MOTOR_INVERTED, ModuleSpecificConstants.BACK_RIGHT_STEER_MOTOR_INVERTED, ModuleSpecificConstants.BACK_RIGHT_ENCODER_INVERTED
        );
        
        // misc
        public static final int TIMEOUT_MS = 30;
        public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
      }
    
    public static final class LiDARConstants {
        public static final int REEF_LIDAR_DIO_CHANNEL = 7;
        public static final int REVERSE_LIDAR_DIO_CHANNEL = 4;

        public static final int HUMAN_PLAYER_STATION_DISTANCE = 40;

        public static final int LIDAR_DETECT_DISTANCE = 100; // Min distance to detect pole
        public static final int LIDAR_MICROS_TO_CM = 10;
        public static final int SECONDS_TO_MICROS = 1000000;
    }

    public static final class AutoConstants {
        // public static final Gains XY_GAINS = new Gains(5,0.6,0.0);
        public static final Gains XY_GAINS = new Gains(8,0,0.0);
        // public static final ConfigurableDouble P_XY_GAINS = new ConfigurableDouble("P_XY_GAINS", XY_GAINS.kP);
        // public static final ConfigurableDouble I_XY_GAINS = new ConfigurableDouble("I_XY_GAINS", XY_GAINS.kI);
        // public static final ConfigurableDouble D_XY_GAINS = new ConfigurableDouble("D_XY_GAINS", XY_GAINS.kD);
       // public static final Gains XY_GAINS = new Gains(3,0.3,0.0);

        // public static final Gains ROT_GAINS = new Gains(0.05,0,0.007);
        // public static final Gains ROT_GAINS = new Gains(0.05,0,0.0);

        public static final Trim X_OFFSET_TRIM =        new Trim("X Offset Trim",        Double.MAX_VALUE, -Double.MAX_VALUE,0.5, 0);
        // public static final Trim Y_OFFSET_TRIM =        new Trim("Y Offset Trim",        Double.MAX_VALUE, -Double.MAX_VALUE, 0.5, 1.5);
        public static final Trim Y_OFFSET_TRIM =        new Trim("Y Offset Trim",        Double.MAX_VALUE, -Double.MAX_VALUE, 0.5, 0);
        public static final Trim ELEVATOR_OFFSET_TRIM = new Trim("Elevator Offset Trim", -ElevatorConstants.MAX_POSITION_ELEVATOR, ElevatorConstants.MAX_POSITION_ELEVATOR, 1, 0);
        public static final Trim ARM_OFFSET_TRIM =      new Trim("ARM Offset Trim",      -ElevatorConstants.COMPLETLY_TOP_ENDEFFECTOR, ElevatorConstants.COMPLETLY_TOP_ENDEFFECTOR, 1, 0);
                
        public static final double XY_TOLERANCE = 0.07; // Meters
        public static final double ROT_TOLERANCE = 5; // Degrees

        public static final double MIN_XY_PID_OUTPUT = 0.0;
        public static final double MIN_ROT_PID_OUTPUT = 0.0;

        public static final double VELOCITY_THRESHHOLD = 0.01;
                
        // X is tangent to reef side
        // Y is normal to reef side
        public static final double X_SCORING_POSITION_OFFSET = Units.inchesToMeters(6.5+1); // This is from the field
        public static final double Y_SCORING_POSITION_OFFSET = Units.inchesToMeters(16+1);
        public static final double HALF_ROBOT_SIZE = Units.inchesToMeters(18);

        public static final double L4_DISTANCE_PREP = HALF_ROBOT_SIZE + Units.inchesToMeters(15);
        public static final double L4_DISTANCE_SCORE = L4_DISTANCE_PREP;
        // public static final double L4_DISTANCE_SCORE = HALF_ROBOT_SIZE + Units.inchesToMeters(4.5);
        
        public static final double L3_DISTANCE_PREP = HALF_ROBOT_SIZE + Units.inchesToMeters(15);
        public static final double L3_DISTANCE_SCORE = HALF_ROBOT_SIZE + Units.inchesToMeters(5+1);
        
        public static final double L2_PREP_DISTANCE = HALF_ROBOT_SIZE + Units.inchesToMeters(6);
        public static final double L2_SCORE_DISTANCE = HALF_ROBOT_SIZE + Units.inchesToMeters(0.5-2);

        public static final double ALGAE_REMOVAL_DISTANCE = HALF_ROBOT_SIZE;
        
        // public static final double L4_DISTANCE_PREP = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(15);
        // public static final double L4_DISTANCE_SCORE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(5.5);
        // // public static final double L4_DISTANCE_SCORE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(4.5);
        
        // public static final double L3_DISTANCE_PREP = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(15);
        // public static final double L3_DISTANCE_SCORE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(5+1);
        
        // public static final double L2_PREP_DISTANCE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(6);
        // public static final double L2_SCORE_DISTANCE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(0.5);

        // public static final double ALGAE_REMOVAL_DISTANCE = Y_SCORING_POSITION_OFFSET + Units.inchesToMeters(2);

        public static final int L4_DRIVE_TIME = 250; //Milliseconds
        // public static final int L3_DRIVE_TIME = 500;
        public static final int L2_DRIVE_TIME = 250; //Milliseconds
        public static final int ALGAE_DRIVE_TIME = 500;
        public static final double STOP_VELOCITY = 0.0;
    }

    public static final class VisionConstants { 
        public static final String LEFT_CAMERA_NAME = "CAMERA_LEFT";
        public static final String RIGHT_CAMERA_NAME = "CAMERA_RIGHT";

        public static final Transform3d LEFT_CAMERA_POS = new Transform3d(new Translation3d(Units.inchesToMeters(4.547), Units.inchesToMeters(8.031), Units.inchesToMeters(8.858)), new Rotation3d(0,0.0,0.0));
        public static final Transform3d RIGHT_CAMERA_POS = new Transform3d(new Translation3d(Units.inchesToMeters(4.547), -Units.inchesToMeters(8.031), Units.inchesToMeters(8.858)), new Rotation3d(0,0.0,0.0));
        
        public static final double MIN_ESTIMATION_DISTANCE = 2; // Meters

        // Photonvision thing
        // The standard deviations of our vision estimated poses, which affect correction rate
        // X, Y, Theta
        // https://www.chiefdelphi.com/t/photonvision-finding-standard-deviations-for-swervedriveposeestimator/467802/2
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.5, 0.5, 4);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 1);
    }

    public static final class FieldConstants {
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // Test april tag field layout
        // public static final AprilTagFieldLayout kTagLayout = new AprilTagFieldLayout(
        //     Arrays.asList(new AprilTag[] {
        //         new AprilTag(1, new Pose3d(
        //             new Translation3d(0.,0.,0.26035), new Rotation3d(0.,0.,0.)
        //         )),
        //     }), 100, 100);

    }

    public static final class DriveConstants {
        public static final int SMARTDASHBOARD_UPDATE_FRAME = 2;
    }
    
    public static final class LEDConstants {
        public static final int LED_SPARK_ID = 9;

        public static final LEDPatterns DEFAULT_PATTERN = LEDPatterns.FOREST_WAVES;

        public static final LEDPatterns WAITING_PATTERN = LEDPatterns.SOLID_RED;
        public static final LEDPatterns DOWN_PATTERN = LEDPatterns.SOLID_YELLOW;
        public static final LEDPatterns READY_PATTERN = LEDPatterns.SOLID_GREEN_DARK;
        public static final LEDPatterns SCORING_PATTERN = LEDPatterns.RAINBOW_RAINBOW;

        public static final LEDPatterns RED_PATTERN = LEDPatterns.LAVA_WAVES;
        public static final LEDPatterns BLUE_PATTERN = LEDPatterns.OCEAN_WAVES;
    }

    public static final class OIConstants {
        public static final int XBOX_DRIVER_ID = 0;
        public static final int XBOX_OPERATOR_ID = 1;
        public static final int BUTTONBOX_ID = 2;
        public static final int XBOX_PROGRAMMER_ID = 3;
        public static final double LEFT_AXIS_DEADBAND = 0.1;

    }

    public static final class ElevatorConstants {
        public static final CanDevice ENDEFFECTOR_ID = new CanDevice("Endeffector", 15);
        public static final CanDevice ELEVATOR_ID = new CanDevice("Elevator", 16);

        public static final double SAFETY_ENDEFFECTOR_MAX_TORQUE = 75;
        public static final double SAFETY_ENDEFFECTOR_MIN_VELOCITY = 20;

        // public static final int BASIN_LIMIT_SWITCH = 8; // TODO: FIND
        
        public static final int BASIN_LIMIT_SWITCH = 8; // TODO: FIND
        public static final int ENDEFFECTOR_LIMIT_SWITCH = 9; // TODO: FIND
        public static final int INTAKE_LIMIT_SWITCH = 6; // TODO: FIND

        
        public static final double GEAR_RATIO_ELEVATOR = -9.0;
        //Max for elevator = 50%
        
        public static final double GROUND_POSITION_ELEVATOR = 0 * GEAR_RATIO_ELEVATOR;
        public static final double WAITING_POSITION_ELEVATOR = -9.5; // TODO: find 2-4 in off the pipe
        public static final double HOVERING_POSITION_ELEVATOR = -7.5; // TODO: find 2-4 in off the pipe
        public static final double WAITING_POSITION_BEAM_BREAK_ELEVATOR = -5; // TODO: find on the pipe
        public static final double SCORING_THREE_ELEVATOR = -9.25;
        public static final double DEALGAE_L2_ELEVATOR = -4;
        public static final double DEALGAE_L3_ELEVATOR = -26.5;
        public static final double MAX_POSITION_ELEVATOR = 4.5 * GEAR_RATIO_ELEVATOR; // TODO: find MAX position
        
        public static final double GEAR_RATIO_ENDEFECTOR = -100.0;
        public static final double ENDEFECTOR_DRIVE_SLOW = -0.08;
        //Max for endefector = 60%
        public static final double L2_SCORE_ELEVATOR = -5;
        public static final double L2_LEAVE_ELEVATOR = -11;

        public static final double L2_SCORE_ENDEFFECTOR = -19;

        public static final double COMPLETLY_DOWN_ENDEFFECTOR = 0 * GEAR_RATIO_ENDEFECTOR;
        public static final double DEALGAE_L2_ENDEFFECTOR = 0.22 * GEAR_RATIO_ENDEFECTOR;
        public static final double COMPLETLY_MIDDLE_ENDEFFECTOR = 0.25 * GEAR_RATIO_ENDEFECTOR;
        public static final double PRIMED_THREE_ENDEFFECTOR = 0.4 * GEAR_RATIO_ENDEFECTOR;
        public static final double SCORING_FOUR_ENDEFFECTOR = 0.3 * GEAR_RATIO_ENDEFECTOR;
        public static final double PRIMED_FOUR_ENDEFFECTOR = 0.44 * GEAR_RATIO_ENDEFECTOR;
        public static final double COMPLETLY_TOP_ENDEFFECTOR = 0.5 * GEAR_RATIO_ENDEFECTOR;

        public static final Slot0Configs ELEVATOR_PID = new Slot0Configs()
            .withKP(1)
            .withKI(0)
            .withKD(0);
        
        public static final Slot0Configs ENDEFFECTOR_PID = new Slot0Configs()
            .withKP(1)
            .withKI(0)
            .withKD(0);
    }
}
