package frc4388.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class ClimberReal implements ClimberIO {
    TalonFX m_climberMotor;
    DigitalInput m_lowerLimit;

    PositionDutyCycle climberPosition = new PositionDutyCycle(0);
    DutyCycleOut climberPercentOutout = new DutyCycleOut(0);

    public ClimberReal(
        TalonFX climberMotor,
        DigitalInput lowerLimit
    ) {
        m_climberMotor = climberMotor;
        m_lowerLimit = lowerLimit;

        // Apply the configs
        m_climberMotor.getConfigurator().apply(ClimberConstants.CLIMBER_PID);
        m_climberMotor.getConfigurator().apply(ClimberConstants.CLIMBER_MOTOR_CONFIG);

        climberPosition.Slot = 0;
    }

    

    @Override
    public void setClimberDistance(ClimberState state, Distance distance) {
        state.climberTargetDistance = distance;

        // length * (motor rot / length) = motor rot
        double angle = distance.in(Inches) * ClimberConstants.CLIMBER_REVS_PER_INCH;

        m_climberMotor.setControl(
            climberPosition.withPosition(Rotations.of(angle))
            .withLimitReverseMotion(m_lowerLimit.get())
        );
    }

    @Override
    public void setPercentOutput(ClimberState state, double percent) {
        // var d = new DutyCycleOut(0);
        m_climberMotor.setControl(
            climberPercentOutout.withOutput(percent)
                .withLimitReverseMotion(m_lowerLimit.get())
            );
    }

    @Override
    public void updateInputs(ClimberState state) {
        // motor rot / (motor rot / length) = length

        double motorRotations = m_climberMotor.getPosition().getValue().in(Rotations);
        double linearInches = motorRotations / ClimberConstants.CLIMBER_REVS_PER_INCH;
        state.climberDistance = Inches.of(linearInches);

        state.climberMotorCurrent = m_climberMotor.getStatorCurrent(false).getValue();
        state.climberLimit = m_lowerLimit.get();

    }

        @Override 
    public void updateGains() {
        ClimberConstants.CLIMBER_PID.kP = ClimberConstants.CLIMBER_kP.get();
        ClimberConstants.CLIMBER_PID.kI = ClimberConstants.CLIMBER_kI.get();
        ClimberConstants.CLIMBER_PID.kD = ClimberConstants.CLIMBER_kD.get();
        m_climberMotor.getConfigurator().apply(ClimberConstants.CLIMBER_PID);
    }
}
