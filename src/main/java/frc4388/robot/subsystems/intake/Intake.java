package frc4388.robot.subsystems.intake;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.subsystems.swerve.SwerveDrive;

public class Intake extends SubsystemBase {
    public IntakeIO io;
    IntakeStateAutoLogged state = new IntakeStateAutoLogged();
    SwerveDrive m_SwerveDrive;
    Supplier<Pose2d> m_swervePoseSupplier;

    public Intake(
        IntakeIO io,
        SwerveDrive m_SwerveDrive
        // Supplier<Pose2d> swervePoseSupplier
    ) {
        this.io = io;
        this.m_SwerveDrive = m_SwerveDrive;
        // this.m_swervePoseSupplier = swervePoseSupplier;
    }

    public enum IntakeMode {
        RollerOn,
        RollerOff,
        ExpelBalls,
        LabubuGrowl,
        Idle
    }
    private boolean overCompressed = false;

    private IntakeMode mode = IntakeMode.Idle;

    public void setMode(IntakeMode mode) {
        this.mode = mode;
    }

    public IntakeMode getMode() {
        return mode;
    }

    public void rollerStop(){
        this.mode = IntakeMode.Idle;
        // io.setRollerOutput(state, 0);
    }

    public double getRollerTarget() {
        return state.rollerTargetOutput;
    }

    public boolean intakeAtReference() {
        return state.extendedSoftLimit;
    }

    public double getRollerSpeed() {
        return state.rollerOutput;
    }
    // public enum FieldZone {
    //     // The robot should aim at the hub
    //     InShootZone,
    //     // The robot should aim towards the wall
    //     AimAtWall,
        
    // }

    // // Calculate what should be done based off of the position of the robot
    // // TODO: Implement field zones
    // public FieldZone getTarget(Pose2d position) {
    //     return FieldZone.InShootZone;
    // }


    @Override
    public void periodic() {
        // FaultReporter.register(this); // TODO Implement fault reporter
        // System.out.println(m_armLimitSwitch.get());
        // ChassisSpeeds chassisSpeeds = m_SwerveDrive.chassisSpeeds;

        // double ChassisOverallSpeed = Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        Logger.processInputs("Intake", state);
        Logger.recordOutput("Intake/IntakeState", this.mode);


        io.updateInputs(state);

        // overCompressed = 
        //     Math.abs(state.armMotorCurrent.in(Amps)) > IntakeConstants.INTAKE_BOUNCE_CURRENT_LIMIT.get();
        //     // Math.abs(state.armMotorVelocity.in(RotationsPerSecond)) < IntakeConstants.INTAKE_BOUNCE_VELOCITY_LIMIT.get();

        // Logger.recordOutput("overCompressed", overCompressed);

        // getCurrentTime

        switch (mode) {
            case RollerOn:
                io.setRollerOutput(state, IntakeConstants.ROLLER_PERCENT_OUTPUT.get());
                break;

            case RollerOff:
                io.setRollerOutput(state, 0);
                break;
                
            case ExpelBalls:
                io.setRollerOutput(state, IntakeConstants.ROLLER_EJECT_PERCENT_OUTPUT.get());
                break;

            case LabubuGrowl:
                io.setRollerOutput(state, IntakeConstants.ROLLER_LABUBU_GROWL_PERCENT_OUTPUT.get());
                break;
            case Idle:
                io.armOutput(0);
                io.setRollerOutput(state, 0);
                break;
        }
        // if (state.retractedLimit){
        //     this.mode = IntakeMode.Retracted;
        // }

    }
}


