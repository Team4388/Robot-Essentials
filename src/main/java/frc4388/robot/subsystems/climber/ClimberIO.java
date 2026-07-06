package frc4388.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

public interface ClimberIO {
    @AutoLog
    public class ClimberState {
        Distance climberDistance = Inches.of(0);
        Distance climberTargetDistance = Inches.of(0);
        Current climberMotorCurrent = Amps.of(0);
        boolean climberLimit = false;
    }

    public default void setClimberDistance(ClimberState state, Distance distance) {}
    public default void setPercentOutput(ClimberState state, double percent) {}

    public default void updateInputs(ClimberState state) {}

    public default void updateGains() {}
}