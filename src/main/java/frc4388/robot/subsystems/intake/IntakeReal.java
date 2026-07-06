package frc4388.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc4388.robot.constants.Constants;
import frc4388.utility.compute.JankCoder;

public class IntakeReal implements IntakeIO {

    SparkMax m_armMotor;
    RelativeEncoder arm_encoder;
    // SparkLimitSwitch reverse_limit;
    TalonFX m_rollerMotor;
    JankCoder m_encoder;
    DigitalInput m_armLimitSwitch;
    boolean m_limitTRIGGER = false;
    private final Timer m_limitTimer = new Timer();


    public IntakeReal(
        DigitalInput armLimitSwitch,
        SparkMax armMotor,
        TalonFX rollerMotor,
        JankCoder jankCoder
    ) {
        // m_angleMotor = angleMotor;
        // m_pitchMotor = pitchMotor;
        m_armMotor = armMotor;
        arm_encoder = m_armMotor.getEncoder();
        m_armLimitSwitch = armLimitSwitch;
        m_rollerMotor = rollerMotor;
        m_encoder = jankCoder;

        m_armMotor.configure(IntakeConstants.ARM_MOTOR_CONFIG, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        m_rollerMotor.getConfigurator().apply(IntakeConstants.ROLLER_MOTOR_CONFIG);
    }


    
    @Override
    public void setRollerOutput(IntakeState state, double rollerOutput) {
        state.rollerTargetOutput = rollerOutput;

        m_rollerMotor.set(rollerOutput);
    }

    @Override
    public void setArmAngle(IntakeState state, Angle angle) {
        
        state.armTargetAngle = angle;
        // Assume that the angle is always accurate, because I think we will use a shaft encoder
        // Assume that 0 degrees = forwards. Might need an offset here

        // angle = clampAng(angle, IntakeConstants.ARM_LIMIT_RETRACTED, IntakeConstants.ARM_LIMIT_EXTENDED);

        // (REAL_ROT) * (MOTOR_ROT / REAL_ROT) = MOTOR_ROT
        Angle motorAngle = angle.times(IntakeConstants.ARM_MOTOR_GEAR_RATIO);
        
        // PositionDutyCycle posRequest = new PositionDutyCycle(motorTargetAngle);
        // m_armMotor.setControl(
        //     armPosition
        //         .withPosition(motorAngle)
        //         .withLimitReverseMotion(!m_armLimitSwitch.get())
        //     );

    }

    @Override
    public void stopArm(){
        m_armMotor.set(0);
        // m_rollerMotor.set(0);
    }

    private boolean retractedLimit() {
        return m_armLimitSwitch.get();
    }
    private boolean retractedSoftLimit() {
        return m_encoder.get() <= IntakeConstants.ARM_LIMIT_RETRACTED.get();
    }
    private boolean extendedLimit() {
        return m_encoder.get() >= IntakeConstants.ARM_LIMIT_EXTENDED.get();
    }

    @Override
    public void armOutput(double percentOutput){

        if(retractedSoftLimit()) {
            percentOutput = Math.max(percentOutput, 0);
        } 
        
        if (extendedLimit()) {
            percentOutput = Math.min(percentOutput, 0);
        }

        m_armMotor.set(percentOutput);
        // System.out.println(percentOutput);

    }

    @Override
    public void armFix(double percentOutput) {
        m_armMotor.set(percentOutput);
    }

    @Override
    public void updateInputs(IntakeState state) {
        m_encoder.update();


        state.armAngle = Rotations.of(arm_encoder.getPosition()).div(IntakeConstants.ARM_MOTOR_GEAR_RATIO);
        state.armMotorVelocity = RotationsPerSecond.of(arm_encoder.getVelocity()).div(IntakeConstants.ARM_MOTOR_GEAR_RATIO);
        // state.armMotorAcceleration = RotationsPerSecondPerSecond.of(m_armMotor.getEncoder().ge);
        state.armMotorCurrent = Amps.of(m_armMotor.getOutputCurrent());

        state.rollerOutput = m_rollerMotor.get();
        state.rollerMotorCurrent = m_rollerMotor.getStatorCurrent().getValue();

        // state.retractedSoftLimit = retractedLimit();
        state.extendedSoftLimit = extendedLimit();

        state.intakeEncoder = m_encoder.getRotations();
        state.encoderConnected = m_encoder.isConnected();

        state.retractedLimitSwitch = retractedLimit();

        // if(state.retractedLimitSwitch && (state.armMotorVelocity.in(RotationsPerSecond) <0)) {
        //     if (!m_limitTRIGGER) {
        //         m_limitTRIGGER = true;
        //         m_limitTimer.restart();
        //     }
        //     if (m_limitTimer.hasElapsed(1.0)) {
        //         m_encoder.resetRotations();
        //     }
        // } else {
        //     m_limitTRIGGER = false;
        //     m_limitTimer.reset();
        // }
    }

    @Override 
    public void updateGains() {
        // m_encoder.loadRotations();


        if(retractedLimit()) {
            m_encoder.resetRotations();
        }

        // IntakeConstants.ARM_PID.kP = IntakeConstants.arm_kP.get();
        // IntakeConstants.ARM_PID.kI = IntakeConstants.arm_kI.get();
        // IntakeConstants.ARM_PID.kD = IntakeConstants.arm_kD.get();
        // m_armMotor.getConfigurator().apply(IntakeConstants.ARM_PID);

    }

    @Override 
    public void fixEncoder() {
        // m_encoder.loadRotations();


        // if(retractedLimit()) {
            m_encoder.resetRotations();
        // }

        // IntakeConstants.ARM_PID.kP = IntakeConstants.arm_kP.get();
        // IntakeConstants.ARM_PID.kI = IntakeConstants.arm_kI.get();
        // IntakeConstants.ARM_PID.kD = IntakeConstants.arm_kD.get();
        // m_armMotor.getConfigurator().apply(IntakeConstants.ARM_PID);

    }
}
