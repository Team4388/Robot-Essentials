package frc4388.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
import frc4388.utility.configurable.ConfigurableDouble;
import frc4388.utility.status.CanDevice;

public class ClimberConstants {
    // Motor conversions
    public static final double CLIMBER_MOTOR_GEAR_RATIO = 1.; // dimentionless
    public static final double CLIMBER_SPOOL_DIAMETER = 1.; // in
    public static final double CLIMBER_REVS_PER_INCH = CLIMBER_MOTOR_GEAR_RATIO / (Math.PI * CLIMBER_SPOOL_DIAMETER); // in

    //IDs
    public static final CanDevice CLIMBER_MOTOR = new CanDevice("Climber Motor", 30);
    public static final int CLIMBER_LIMIT_SWITCH_CHANNEL = 8;

    // Constants
    public static final Distance CLIMBER_LIMIT_LOWER = Inches.of(0);
    public static final ConfigurableDouble CLIMBER_LIMIT_UPPER = new ConfigurableDouble("Climber Height in", 6.);
    public static final ConfigurableDouble CLIMBER_RETRACT_SPEED = new ConfigurableDouble("Climber retract %", 0.3);

    public static final Slot0Configs CLIMBER_PID = new Slot0Configs()
        .withKP(0.3)
        .withKI(0.0)
        .withKD(0.0);

    public static final ConfigurableDouble CLIMBER_kP = new ConfigurableDouble("Climber kP", 0.3);
    public static final ConfigurableDouble CLIMBER_kI = new ConfigurableDouble("Climber kI", 0.);
    public static final ConfigurableDouble CLIMBER_kD = new ConfigurableDouble("Climber kD", 0.);
    
    // Motor configs
    public static final TalonFXConfiguration CLIMBER_MOTOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40) // TODO: tune???
                .withStatorCurrentLimitEnable(true)
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake) // Must be break because this has to be accurate
                    .withDutyCycleNeutralDeadband(0.04) // This sets the minimum output of motor so if its less than 4% it wont move
    );
}