package frc4388.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;

import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc4388.utility.configurable.ConfigurableDouble;
import frc4388.utility.status.CanDevice;

public class IntakeConstants {
    // Motor conversions

    public static final double ARM_MOTOR_GEAR_RATIO = 125;
    public static final double ROLLER_MOTOR_GEAR_RATIO = 3;
    public static final ConfigurableDouble ARM_ENCODER_OFFSET = new ConfigurableDouble("Arm Encoder Offset", 0);

    public static final int ARM_LIMIT_SWITCH_CHANNEL = 7;

    public static final ConfigurableDouble INTAKE_BOUNCE_HALF_PERIOD = new ConfigurableDouble("Bounce Half Period", 5.);
    public static final ConfigurableDouble INTAKE_BOUNCE_OUTPUT = new ConfigurableDouble("Bounce Output", 0.1);
    public static final ConfigurableDouble INTAKE_BOUNCE_MAX_OUTPUT = new ConfigurableDouble("Bounce Max Output", 0.2);
    public static final ConfigurableDouble INTAKE_BOUNCE_CURRENT_LIMIT = new ConfigurableDouble("Intake Bounce Current Limit", 30);
    public static final ConfigurableDouble INTAKE_BOUNCE_VELOCITY_LIMIT = new ConfigurableDouble("Intake Bounce Velocity Limit", 4);


    //squeeze constants
    // public static final ConfigurableDouble INTAKE_SQUEEZE_CURRENT_LOWER_THRESHOLD = new ConfigurableDouble("Intake Squeeze Current LOWER THRESHOLD", 20);
    // public static final ConfigurableDouble INTAKE_SQUEEZE_CURRENT_UPPER_THRESHOLD = new ConfigurableDouble("Intake Squeeze Current UPPER THRESHOLD", 25);
    public static final ConfigurableDouble ARM_SQUEEZE_PERCENT_OUTPUT = new ConfigurableDouble("Arm squeeze % output", -0.2);
    public static final ConfigurableDouble ARM_REDUCED_SQUEEZE_PERCENT_OUTPUT = new ConfigurableDouble("Arm reduce squeeze % output", 0.02);
    public static final ConfigurableDouble FIX_ARM_PERCENT_OUTPUT = new ConfigurableDouble("Arm encoder fix % output", -0.1);

    //IDs

    public static final CanDevice ARM_ID   = new CanDevice("ARM", 20);
    public static final CanDevice ROLLER_ID   = new CanDevice("INTAKE_ROLLER", 21);
    public static final int ARM_ENCODER_ID = 0;
        
    // Limits

    // 0 is the forward angle on the robot.
    // negative is left, positive is right

    //when testing the negative output of 10% made the robot put the intake up

    // public static final Angle ARM_LIMIT_LOWER = Degrees.of(90);
    // public static final Angle ARM_LIMIT_UPPER = Degrees.of(-90);

    public static final ConfigurableDouble ARM_LIMIT_RETRACTED = new ConfigurableDouble("Arm angle retracted", 0.);
    public static final ConfigurableDouble ARM_LIMIT_EXTENDED = new ConfigurableDouble("Arm angle extended", 1.625); //new soft limt
    public static final ConfigurableDouble ARM_EXTEND_PERCENT_OUTPUT = new ConfigurableDouble("Arm extend % output", 0.3);
    public static final ConfigurableDouble ARM_RETRACT_PERCENT_OUTPUT = new ConfigurableDouble("Arm retract % output", -0.15);

    public static final ConfigurableDouble ARM_AUTO_OUTPUT = new ConfigurableDouble("Arm auto output", -0.07);
    public static final ConfigurableDouble ARM_REVERSE_ROLLER_RANGE = new ConfigurableDouble("Arm reverse roller range", 1.17);
    
    public static final ConfigurableDouble ROLLER_PERCENT_OUTPUT = new ConfigurableDouble("Roller Percent Output", .50);
    public static final ConfigurableDouble ROLLER_LABUBU_GROWL_PERCENT_OUTPUT = new ConfigurableDouble("Roller Labubu Growl Percent Output", .7);
    public static final ConfigurableDouble ROLLER_EJECT_PERCENT_OUTPUT = new ConfigurableDouble("Roller eject Percent Output", -.20);
    public static final ConfigurableDouble ROLLER_IDLE_PERCENT_OUTPUT = new ConfigurableDouble("Roller IDLE Percent Output", .20);
    public static final ConfigurableDouble ROLLER_RETRACT_PERCENT_OUTPUT = new ConfigurableDouble("Roller Retract Output", .40);
    public static final ConfigurableDouble ROLLER_MULTIPLIER_CONST = new ConfigurableDouble("Roller Multiplier Constant", 0.4);

    // public static final ConfigurableDouble ROLL = new ConfigurableDouble("Arm angle extended", 0.25);

    // public static final AngularVelocity ROLLER_MAX_VELOCITY = RotationsPerSecond.of(4.0);
    // public static final AngularVelocity ROLLER_STOP = RotationsPerSecond.of(0.0);


    // public static final Slot0Configs ARM_PID = new Slot0Configs()
    //     .withKP(0.08)
    //     .withKI(0.0)
    //     .withKD(0.05);

    

    // public static ConfigurableDouble arm_kP = new ConfigurableDouble("ARM KP", 0.08);
    // public static ConfigurableDouble arm_kI = new ConfigurableDouble("ARM KI", 0);
    // public static ConfigurableDouble arm_kD = new ConfigurableDouble("ARM KD", 0.05);
    
    public static double getTargetRollerSpeed(double chassisSpeed) {
        return ROLLER_PERCENT_OUTPUT.get() * (1 + ROLLER_MULTIPLIER_CONST.get() * chassisSpeed);
    }


    // 0 is paralell to the ground, 90 is directly up
    // public static final Angle PITCH_LIMIT_UPPER = Degrees.of(90);
    // public static final Angle PITCH_LIMIT_LOWER = Degrees.of(0);
    
    // Motor configs

    public static final SparkMaxConfig ARM_MOTOR_CONFIG = new SparkMaxConfig();
    public static final CanDevice ROLLER_MOTOR_ID = new CanDevice("INTAKE_ROLLER", 21);

    static {
        ARM_MOTOR_CONFIG.limitSwitch
            .limitSwitchPositionSensor(FeedbackSensor.kPrimaryEncoder)

            .forwardLimitSwitchType(Type.kNormallyOpen)
            .forwardLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor)

             .reverseLimitSwitchType(Type.kNormallyOpen)
            // .reverseLimitSwitchPosition(0)
            .reverseLimitSwitchTriggerBehavior(Behavior.kKeepMovingMotor);

        ARM_MOTOR_CONFIG.idleMode(IdleMode.kBrake);

        
        // ROLLER_MOTOR_CONFIG.idleMode(IdleMode.kCoast);
    }

    // public static final TalonFXConfiguration ARM_MOTOR_CONFIG = new TalonFXConfiguration()
    //     .withCurrentLimits(
    //         new CurrentLimitsConfigs()
    //             .withStatorCurrentLimit(15) // TODO: tune???
    //             .withStatorCurrentLimitEnable(true)
    //         ).withMotorOutput(
    //             new MotorOutputConfigs()
    //                 .withNeutralMode(NeutralModeValue.Brake) // Must be break because this has to be accurate
    //                 .withDutyCycleNeutralDeadband(0.04) // This sets the minimum output of motor so if its less than 4% it wont move
    // );

    public static final TalonFXConfiguration ROLLER_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // Must be coast because this is spinny spinny
                    .withDutyCycleNeutralDeadband(0.04) // This sets the minimum output of motor so if its less than 4% it wont move
    );
}
