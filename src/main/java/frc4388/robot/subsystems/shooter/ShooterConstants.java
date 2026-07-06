package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import frc4388.utility.configurable.ConfigurableDouble;
import frc4388.utility.status.CanDevice;

public class    ShooterConstants {
    // Motor conversions
    
    public static final double SHOOTERMOTOR_GEAR_RATIO = 1.286; // TODO: supposed to be 9 rotations in to 7 out    -- 0.77 or 1.29
    // public static final double INDEXER_GEAR_RATIO = 1.;
    public static final double T_CONSTANT = 2;
    public static final double SHOOTER_RADIUS = 2/39.37;
    public static final double INDEXER_RADIUS = 0.625/39.37;
    public static final ConfigurableDouble SHOOTER_MAX_VELOCITY = new ConfigurableDouble("Shooter MAX Velocity", 60);
    public static final ConfigurableDouble SHOOTER_OVERRIDE_VELOCITY = new ConfigurableDouble("Shooter OVERRIDE Velocity", -28);
    // public static final ConfigurableDouble SHOOTER_FEED_VELOCITY = new ConfigurableDouble("Shooter Feed Velocity", -35);
    // public static final ConfigurableDouble SHOOTER_RESTING_VELOCITY = new ConfigurableDouble("Shooter Resting Velocity", 0.0);
    
    
    public static final ConfigurableDouble SHOOTER_IDLE_PERCENT_OUTPUT = new ConfigurableDouble("Shooter idle % output", 0.0);
    public static final ConfigurableDouble DEMO_PERCENT_OUTPUT = new ConfigurableDouble("Demo % output", 45);
    // public static final ConfigurableDouble SHOOTER_IDLE_TARGET_VEL = new ConfigurableDouble("Shooter idle target velocity", 20.);
    // public static final ConfigurableDouble SHOOTER_IDLE_MAX_CURRENT = new ConfigurableDouble("Shooter Idle max current", 10);

    public static final ConfigurableDouble INDEXER_FORWARD_OUTPUT = new ConfigurableDouble("Indexer FWD % Output", -0.4);
    public static final ConfigurableDouble INDEXER_REVERSE_OUTPUT = new ConfigurableDouble("Indexer reverse % Output", 0.2);
    public static final ConfigurableDouble MODEL_TRIM = new ConfigurableDouble("TRIM SHOOTER SPEED", 0);

    
    public static final ConfigurableDouble NEG_OFFSET = new ConfigurableDouble("Negative offset", 8.);
    public static final ConfigurableDouble POS_OFFSET = new ConfigurableDouble("Positive offset", 8.);

    
    public static final ConfigurableDouble AIM_LEAD_TIME = new ConfigurableDouble("Aim lead time", -1.1);


    // Shoot mode tolerances
    public static final ConfigurableDouble ROBOT_MIN_HUB = new ConfigurableDouble("Shoot min dist M", 1.8);
    public static final ConfigurableDouble ROBOT_MAX_HUB = new ConfigurableDouble("Shoot max dist M", 4.8);

    public static final ConfigurableDouble AIM_ANGLE = new ConfigurableDouble("Aim angle tolerance", 15);
    public static final ConfigurableDouble ROBOT_ANG_TOLERANCE = new ConfigurableDouble("Ang tolerance DEG", 360);

    public static final ConfigurableDouble ROBOT_SPEED_TOLERANCE = new ConfigurableDouble("Speed tolerance MS", 1);
    public static final ConfigurableDouble ROBOT_ANG_SPEED_TOLERANCE = new ConfigurableDouble("Shoot Ang speed tolerance DEG", 3);

    public static final ConfigurableDouble SHOOTER_SPEED_TOLERANCE = new ConfigurableDouble("Shooter speed tolerance RPS", 3); 

    // 
    public static AngularVelocity getTargetShooterSpeed(double hubDistMeters, double chassisXSpeed) {
        double speed = DEMO_PERCENT_OUTPUT.get();

        // if (Math.abs(chassisXSpeed) < 0.1){
        //     speed = 0.0593402*hubDistMeters*hubDistMeters +
        //     4.90561*hubDistMeters +
        //     30.35696 + MODEL_TRIM.get();
        // } else if (chassisXSpeed > 0){
        //     speed = 0.0593402*hubDistMeters*hubDistMeters +
        //     4.90561*hubDistMeters +
        //     30.35696 + chassisXSpeed * POS_OFFSET.get() + MODEL_TRIM.get();
            
        // } else { // Negative is closer to hub
        //     speed = 0.0593402*hubDistMeters*hubDistMeters +
        //     4.90561*hubDistMeters +
        //     30.35696 + chassisXSpeed * NEG_OFFSET.get() + MODEL_TRIM.get();
        // }
        
        // double max = SHOOTER_MAX_VELOCITY.get();

        // // Clamp speed to max
        // if(speed > max) {
        //     speed = max;
        // } else if(speed < -max) {
        //     speed = -max;
        // }

        // double speed = SHOOTER_MAX_VELOCITY.get();
        
        return RotationsPerSecond.of(-speed);
    }

    // Motor Configuration
    public static Slot0Configs SHOOTER_PID = new Slot0Configs()
        .withKV(0.0)
        .withKP(0.02)
        .withKI(0.15)
        .withKD(0.0);

    
    
    
    
    
    public static ConfigurableDouble shooter_kP = new ConfigurableDouble("Shooter KP", 0.02);
    public static ConfigurableDouble shooter_kI = new ConfigurableDouble("Shooter KI", 0.15);
    public static ConfigurableDouble shooter_kD = new ConfigurableDouble("Shooter KD", 0);


    public static final CanDevice SHOOTER1_ID = new CanDevice("SHOOTER 1", 22);
    public static final CanDevice SHOOTER2_ID = new CanDevice("SHOOTER 2", 23);
    public static final CanDevice INDEXER_ID  = new CanDevice("INDEXER",24);
    

    public static final TalonFXConfiguration SHOOTER1_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // Must be coast because this is spinny spinny
                    .withDutyCycleNeutralDeadband(0.04) // This sets the minimum output of motor so if its less than 4% it wont move
    );
    public static final TalonFXConfiguration SHOOTER2_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true) // TODO: Figure out what this means
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast) // Must be coast because this is spinny spinny
                    .withDutyCycleNeutralDeadband(0.04) // This sets the minimum output of motor so if its less than 4% it wont move
    );

    public static final TalonFXConfiguration INDEXER_MOTOR_CONFIG = new TalonFXConfiguration()
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
