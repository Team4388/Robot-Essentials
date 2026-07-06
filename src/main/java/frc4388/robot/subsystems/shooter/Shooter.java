package frc4388.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.constants.Constants;
import frc4388.robot.subsystems.intake.Intake;
import frc4388.robot.subsystems.led.LED;
import frc4388.robot.subsystems.swerve.SwerveDrive;
import frc4388.utility.compute.FieldPositions;
import frc4388.utility.compute.HubShiftTimer;
import frc4388.utility.compute.HubShiftTimer.ShiftInfo;
import frc4388.utility.compute.TimesNegativeOne;

public class Shooter extends SubsystemBase {
    public ShooterIO io;
    ShooterStateAutoLogged state = new ShooterStateAutoLogged();

    SwerveDrive m_SwerveDrive;
    Intake m_Intake;
    LED m_robotLED;
    

    // Supplier<Pose2d> m_swervePoseSupplier;
    public boolean badShooterVelocity;
    public double distanceToHub = 5;
    public double chassisXSpeed = 0;



    public Shooter(
        ShooterIO io,
        SwerveDrive swerveDrive,
        Intake intake,
        LED robotLED
    ) {
        this.io = io;
        this.m_SwerveDrive = swerveDrive;
        this.m_Intake = intake;
        this.m_robotLED = robotLED;
        // this.m_swervePoseSupplier = swervePoseSupplier;
    }

    public enum FieldZone {
        // The robot should aim at the hub
        InShootZone,
        // The robot should aim towards the wall
        AimAtWall,
        
    }

    
    public enum ShooterMode {
        Shooting,
        ManualShoot,
        Idle
    }

    private ShooterMode mode = ShooterMode.Idle;
    private boolean shooterButtonReady = false;

    public void spinUpShooting() {
        this.mode = ShooterMode.Shooting;
    }

    public void spinUpFeeding() {
        this.mode = ShooterMode.ManualShoot;
    }
    
    public void spinUpIdle() {
        this.mode = ShooterMode.Idle;
    }

    public double getDistanceToHub(){
        return distanceToHub;
    }

    public void allowShooting() {
        shooterButtonReady = true;
    }   

    public void denyShooting() {
        shooterButtonReady = false;
    }

    public double getBallVelocity() {
        return Math.abs(state.motor1TargetVelocity.in(RotationsPerSecond)) * ShooterConstants.SHOOTER_RADIUS * 2 * Math.PI;
        //Math.abs(state.indexerForwardVelocity.in(RotationsPerSecond))*ShooterConstants.INDEXER_RADIUS)
    }

    @AutoLogOutput
    public ShooterMode getMode() {
        return mode;
    }

    @AutoLogOutput
    public boolean isShooterUpToSpeed() {
        return !badShooterVelocity;
    }



    @Override
    public void periodic() {
        // FaultReporter.register(this); // TODO Implement fault reporter
        //Hub Shift logs
        ShiftInfo info = HubShiftTimer.getShiftInfo();
        Logger.recordOutput("HubShift/IsActive", info.isActive());
        Logger.recordOutput("HubShift/RemainingInShift", info.remainingInShift());
        Logger.recordOutput("HubShift/Phase", info.phase().name());
        Logger.processInputs("Shooter", state);

        io.updateInputs(state);


        // Get robot positon and speeds
        ChassisSpeeds chassisSpeeds = m_SwerveDrive.chassisSpeeds;
        Translation2d robotSpeed = new Translation2d(
            chassisSpeeds.vxMetersPerSecond, 
            chassisSpeeds.vyMetersPerSecond
        );
        Translation2d fieldPosLead = robotSpeed.times(ShooterConstants.AIM_LEAD_TIME.get()).plus(FieldPositions.HUB_POSITION);
        Rotation2d ang = m_SwerveDrive.getPose2d().getTranslation().minus(fieldPosLead).getAngle().minus(m_SwerveDrive.getPose2d().getRotation());
        Pose2d robotPose2d = m_SwerveDrive.getPose2d();

        if (TimesNegativeOne.isRed) {
            chassisXSpeed = chassisSpeeds.vxMetersPerSecond;
        } else {
            chassisXSpeed = -chassisSpeeds.vxMetersPerSecond;
        }

        Translation2d fieldPos = robotPose2d.getTranslation();
        // Get the robot's aim distance to hub
        distanceToHub = (fieldPos.minus(FieldPositions.HUB_POSITION).getNorm());

        //Center of hub to cameras in meters
        Logger.recordOutput("Hub Dist", distanceToHub);

        boolean driverError = 
            // XYSpeed <= ShooterConstants.ROBOT_SPEED_TOLERANCE.get() |
            // AngSpeed <= ShooterConstants.ROBOT_ANG_SPEED_TOLERANCE.get() |
            distanceToHub <= ShooterConstants.ROBOT_MIN_HUB.get() | 
            distanceToHub >= ShooterConstants.ROBOT_MAX_HUB.get() |
            Math.abs(ang.getDegrees()) > ShooterConstants.AIM_ANGLE.get();


        double shooterSpeed = Math.abs(state.motor1Velocity.in(RotationsPerSecond) + state.motor2Velocity.in(RotationsPerSecond)) / 2;
        double shooterSpeedTarget = Math.abs(state.motor1TargetVelocity.in(RotationsPerSecond) + state.motor2TargetVelocity.in(RotationsPerSecond)) / 2;

        badShooterVelocity = Math.abs(shooterSpeed - shooterSpeedTarget) > ShooterConstants.SHOOTER_SPEED_TOLERANCE.get();


        switch (mode) {
            case Shooting:
                io.setShooterVelocity(state, ShooterConstants.getTargetShooterSpeed(distanceToHub, chassisXSpeed));

                int bitmask = (
                    (shooterButtonReady ? 1 : 0) +
                    (badShooterVelocity ? 2 : 0) +
                    (driverError ? 4 : 0)
                );

                switch (bitmask) {
                    case 0b000: // No errors but button is not pressed
                        io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                        m_robotLED.setMode(Constants.LEDConstants.OPREADY);
                        break;

                    case 0b001: // No errors and shoot button is pressed     
                        io.setIndexerOutput(state, ShooterConstants.INDEXER_FORWARD_OUTPUT.get());
                        m_robotLED.setMode(Constants.LEDConstants.OPREADY);
                        break;

                    case 0b010: // Bad shooter velocity, button is not pressed
                    case 0b011: // Bad shooter velocty, button is pressed
                        io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                        m_robotLED.setMode(Constants.LEDConstants.BAD_FLYWEEL);
                        break;

                    case 0b100: // Driver error, button is not pressed
                    case 0b101: // Driver error, button is pressed
                        io.setIndexerOutput(state, 0);
                        m_robotLED.setMode(Constants.LEDConstants.OPREADY_BADPHYS);
                        break;

                    case 0b110: // Driver error, bad shooter vel, button is not pressed
                    case 0b111: // Driver error, bad shooter vel, button is pressed
                        io.setIndexerOutput(state, 0);
                        m_robotLED.setMode(Constants.LEDConstants.BAD_FLYWEEL_BADPHYS);
                        break;
                }
                break;
            case ManualShoot:
                if(io.demoSpeed(state)){
                    io.setIndexerOutput(state, ShooterConstants.INDEXER_FORWARD_OUTPUT.get());
                }else{
                    io.setIndexerOutput(state, ShooterConstants.INDEXER_REVERSE_OUTPUT.get());
                }
                io.setShooterVelocity(state, ShooterConstants.getTargetShooterSpeed(distanceToHub, chassisXSpeed));
                m_robotLED.setMode(Constants.LEDConstants.OPREADY_FEED);
                break;
            case Idle:

                io.setShooterCurrentLimitSpeed(
                    state, 
                    ShooterConstants.SHOOTER_IDLE_PERCENT_OUTPUT.get()
                    // Amps.of(ShooterConstants.SHOOTER_IDLE_MAX_CURRENT.get()), 
                    // RotationsPerSecond.of(ShooterConstants.INDEXER_REVERSE_OUTPUT.get())
                );
                io.setIndexerOutput(state, 0);
                m_robotLED.setMode(Constants.LEDConstants.DEFAULT_PATTERN);
                break;
            }
            
        // io.motorStalled(state, m_Intake, m_robotLED);
    }
}
