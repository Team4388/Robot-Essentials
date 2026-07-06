package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import frc4388.robot.constants.Constants;
import frc4388.robot.subsystems.intake.Intake;
import frc4388.robot.subsystems.led.LED;

public class ShooterReal implements ShooterIO {

    TalonFX m_shooter1Motor;
    TalonFX m_shooter2Motor;
    TalonFX m_indexerMotor;

    VelocityDutyCycle shooter1Velocity = new VelocityDutyCycle(0);
    VelocityDutyCycle shooter2Velocity = new VelocityDutyCycle(0);
    // VelocityDutyCycle m_indexerVelocity = new VelocityDutyCycle(0);

    private final Timer m_stallTimerShooter = new Timer();
    private final Timer m_stallTimerIndexer = new Timer();
    private final Timer m_stallTimerRoller = new Timer();
    private boolean m_shooterStalling = false;
    private boolean m_indexerStalling = false;
    private boolean m_rollerStalling = false;
    public String motorStall = "";


    public ShooterReal(
         TalonFX shooter1Motor,
         TalonFX shooter2Motor,
         TalonFX indexerMotor
    ) {
        m_shooter1Motor = shooter1Motor;
        m_shooter2Motor = shooter2Motor;
        m_indexerMotor  = indexerMotor;

        m_shooter1Motor.getConfigurator().apply(ShooterConstants.SHOOTER_PID);
        m_shooter2Motor.getConfigurator().apply(ShooterConstants.SHOOTER_PID);

        m_shooter1Motor.getConfigurator().apply(ShooterConstants.SHOOTER1_MOTOR_CONFIG);
        m_shooter2Motor.getConfigurator().apply(ShooterConstants.SHOOTER2_MOTOR_CONFIG);
        m_indexerMotor.getConfigurator().apply(ShooterConstants.INDEXER_MOTOR_CONFIG);

        shooter1Velocity.Slot = 0;
        shooter2Velocity.Slot = 0;
        // m_indexerVelocity.Slot = 0;
    }
    
    @Override
    public void motorStalled(ShooterState state, Intake m_Intake, LED m_robotLED) {    
            motorStall = "";
        if (Math.abs(state.motor1TargetVelocity.in(RotationsPerSecond)) - Math.abs(state.motor1Velocity.in(RotationsPerSecond)) > 40) {
            if (!m_shooterStalling) {
                m_shooterStalling = true;
                m_stallTimerShooter.restart();
            }
            if (m_stallTimerShooter.hasElapsed(5.0)) {
                m_robotLED.setMode(Constants.LEDConstants.MOTOR_STALLED);
                motorStall = "Shooter Stalled";
                System.out.println("Shooter Stalled: " + (Math.abs(state.motor1TargetVelocity.in(RotationsPerSecond)) - Math.abs(state.motor1Velocity.in(RotationsPerSecond))));
                System.out.println("Target Velocity: " + Math.abs(state.motor1TargetVelocity.in(RotationsPerSecond)));
                System.out.println("Actual Velocity: " + Math.abs(state.motor1Velocity.in(RotationsPerSecond)));
            }
        } else {
            m_shooterStalling = false;
            m_stallTimerShooter.reset();
        }

        if (Math.abs(state.indexerTargetOutput) - Math.abs(state.indexerOutput) > 0.3) {
            if (!m_indexerStalling) {
                m_indexerStalling = true;
                m_stallTimerIndexer.restart();
            }
            if (m_stallTimerIndexer.hasElapsed(5.0)) {
                m_robotLED.setMode(Constants.LEDConstants.MOTOR_STALLED);
                motorStall = "Indexer Stalled";
                System.out.println("Indexer Stalled: " + (Math.abs(state.indexerTargetOutput) - Math.abs(state.indexerOutput)));
            }
        } else {
            m_indexerStalling = false;
            m_stallTimerIndexer.reset();
        }

        if (Math.abs(m_Intake.getRollerTarget()) - Math.abs(m_Intake.getRollerSpeed()) > 0.4) {
            if (!m_rollerStalling) {
                m_rollerStalling = true;
                m_stallTimerRoller.restart();
            }
            if (m_stallTimerRoller.hasElapsed(5.0)) {
                m_robotLED.setMode(Constants.LEDConstants.MOTOR_STALLED);
                motorStall = "Roller Stalled";
                System.out.println("Roller Stalled: " + (Math.abs(m_Intake.getRollerTarget()) - Math.abs(m_Intake.getRollerSpeed())));
            }
        } else {
            m_rollerStalling = false;
            m_stallTimerRoller.reset();
        }
        Logger.recordOutput("Stalled Motor: ", motorStall);
        
    }
    
    @Override
    public void setShooterVelocity(ShooterState state, AngularVelocity target) {
        state.motor1TargetVelocity = target;
        state.motor2TargetVelocity = target;

        if(target.baseUnitMagnitude() == 0) {
            m_shooter1Motor.set(0);
            m_shooter2Motor.set(0);
            return;
        }

        AngularVelocity motorRps = target.times(ShooterConstants.SHOOTERMOTOR_GEAR_RATIO);

        m_shooter1Motor.setControl(shooter1Velocity.withVelocity(motorRps));
        m_shooter2Motor.setControl(shooter2Velocity.withVelocity(motorRps));
    }

    @Override
    public void setShooterCurrentLimitSpeed(
        ShooterState state, 
        double percentOutput
        // Current currentLimit,
        // AngularVelocity targetVelocity
    ) {
        // state.motor1TargetVelocity = targetVelocity;
        // state.motor2TargetVelocity = targetVelocity;

        // double current = Math.abs(state.motor1Current.in(Amps)) + Math.abs(state.motor2Current.in(Amps));
        // double velocity = (Math.abs(state.motor1Velocity.in(RotationsPerSecond)) + Math.abs(state.motor2Velocity.in(RotationsPerSecond))) / 2;

        // if(
        //     Math.abs(currentLimit.in(Amps)) > current &&
        //     Math.abs(targetVelocity.in(RotationsPerSecond)) > velocity
        // ) {
            state.motor1TargetVelocity = RotationsPerSecond.of(percentOutput);
            state.motor2TargetVelocity = RotationsPerSecond.of(percentOutput);
            m_shooter1Motor.set(percentOutput);
            m_shooter2Motor.set(percentOutput);
        // } else {
        //     m_shooter1Motor.set(0);
        //     m_shooter2Motor.set(0);
        // }
    }

    @Override
    public void setIndexerOutput(ShooterState state, double percentOutput) {
        state.indexerTargetOutput = percentOutput;
        m_indexerMotor.set(percentOutput);
    }
    
    // @Override
    // public boolean demoSpeed(ShooterState state) {
    //     boolean demo = false;
    //     if((Math.abs(Math.abs(state.motor1TargetVelocity.in(RotationsPerSecond))-Math.abs(state.motor2TargetVelocity.in(RotationsPerSecond))) < 3)){
    //         demo = true;
    //     }
    //     return demo;
    // }


    @Override
    public void updateInputs(ShooterState state) {

        state.motor1Velocity = m_shooter1Motor.getVelocity().getValue().div(ShooterConstants.SHOOTERMOTOR_GEAR_RATIO);
        state.motor2Velocity = m_shooter2Motor.getVelocity().getValue().div(ShooterConstants.SHOOTERMOTOR_GEAR_RATIO);
        state.indexerOutput = m_indexerMotor.get();
        // state.indexerOutput = m_indexerMotor.getVelocity().getValue().div(ShooterConstants.INDEXER_GEAR_RATIO);

        // state.motorLinearVelocity = InchesPerSecond.of(m_shooter1Motor.getVelocity().getValue().in(RotationsPerSecond) * ShooterConstants.FEEDER_INCHES_PER_ROT);
        
        state.motor1Current = m_shooter1Motor.getStatorCurrent().getValue();
        state.motor2Current = m_shooter2Motor.getStatorCurrent().getValue();
        state.indexerCurrent = m_indexerMotor.getStatorCurrent().getValue();

    }

    @Override 
    public void updateGains() {
        // TEMPORARY PIDs
        ShooterConstants.SHOOTER_PID.kP = ShooterConstants.shooter_kP.get();
        ShooterConstants.SHOOTER_PID.kI = ShooterConstants.shooter_kI.get();
        ShooterConstants.SHOOTER_PID.kD = ShooterConstants.shooter_kD.get();
        m_shooter1Motor.getConfigurator().apply(ShooterConstants.SHOOTER_PID);
        m_shooter2Motor.getConfigurator().apply(ShooterConstants.SHOOTER_PID);
    }
    
}
