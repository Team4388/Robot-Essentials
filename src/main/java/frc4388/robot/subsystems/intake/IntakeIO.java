package frc4388.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public interface IntakeIO {
    @AutoLog
    public class IntakeState {
        double currentBounceTime = 0;

        boolean retractedLimitSwitch = false;
        boolean extendedSoftLimit = false;
        boolean retractedSoftLimit = false;

        boolean encoderConnected = false;
        Angle intakeEncoder = Rotations.of(0);

        Angle armAngle = Rotations.of(0);
        Angle armTargetAngle = Rotations.of(0);
        Current armMotorCurrent = Amps.of(0);


        AngularVelocity armMotorVelocity = RotationsPerSecond.of(0);
        // AngularAcceleration armMotorAcceleration = RotationsPerSecondPerSecond.of(0);

        // Angle shooterPitch = Rotations.of(0);
        // Angle shooterTargetPitch = Rotations.of(0);
        // Current pitchMotorCurrent = Amps.of(0);

        double rollerOutput = 0;
        double rollerTargetOutput = 0;
        Current rollerMotorCurrent = Amps.of(0);

        // LinearVelocity feederVelocity = InchesPerSecond.of(0);
        // LinearVelocity feederTargetVelocity = InchesPerSecond.of(0);
        // Current feederMotorCurrent = Amps.of(0);
    }

    // public default void setShooterAngle(ShooterState state, Angle angle) {}
    // public default void setShooterPitch(ShooterState state, Angle angle) {}
    public default void setArmAngle(IntakeState state, Angle angle) {}
    public default void testSetArmAngle(IntakeState state, Angle angle){}
    public default void stopArm() {}
    public default void setRollerOutput(IntakeState state, double rollerOutput) {}
    public default void armOutput(double percentOutput) {}
    public default void armFix(double percentOutput) {}
    public default void updateInputs(IntakeState state) {}
    public default void updateGains() {}
    public default void fixEncoder() {}
}