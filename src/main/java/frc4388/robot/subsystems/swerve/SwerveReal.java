package frc4388.robot.subsystems.swerve;

import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc4388.robot.subsystems.vision.Vision;
import frc4388.robot.subsystems.vision.VisionIO.PoseObservation;
import frc4388.utility.status.CanDevice;

public class SwerveReal implements SwerveIO {
    SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain;

    public SwerveReal(SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerveDriveTrain) {
        this.swerveDriveTrain = swerveDriveTrain;
        swerveDriveTrain.getOdometryFrequency();
    }

    @Override
    public void updateInputs(SwerveState state) {
        double time = Vision.getTime();
        state.odometryRate = 1 / swerveDriveTrain.getOdometryFrequency();
        state.currentPose = swerveDriveTrain.samplePoseAt(time).orElse(null);
        state.lastPose = swerveDriveTrain.samplePoseAt(time - state.odometryRate).orElse(null);
        state.speeds = swerveDriveTrain.getState().Speeds;
    }

    @Override
    public void setControl(SwerveRequest ctrl) {
        swerveDriveTrain.setControl(ctrl);
    }

    @Override
    public void tareEverything() {
        swerveDriveTrain.tareEverything();
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (pose == null) return;
        try {
            // Preferred: ask the drivetrain to reset its odometry directly
            System.out.println("!"+pose);
            swerveDriveTrain.resetPose(pose);
        } catch (NoSuchMethodError | RuntimeException e) {
            // Fallback: tare sensors then add a timed vision measurement so odometry is seeded
            swerveDriveTrain.tareEverything();
            swerveDriveTrain.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(Vision.getTime()));
        }
    }
    
    @Override
    public void setLimits(double limitInAmps) {
        for (SwerveModule<TalonFX, TalonFX, CANcoder> module : swerveDriveTrain.getModules()) {
            var talonFXConfigurator = module.getDriveMotor().getConfigurator();
            var talonFXConfigs = new TalonFXConfiguration();

            talonFXConfigurator.refresh(talonFXConfigs);
            talonFXConfigs.CurrentLimits.StatorCurrentLimit = limitInAmps;
            talonFXConfigs.CurrentLimits.SupplyCurrentLimit = limitInAmps+10;
            talonFXConfigurator.apply(talonFXConfigs);
        }
    }

    @Override
    public void addVisionMeasurement(List<PoseObservation> poses) {
        for(PoseObservation pose : poses) {
            swerveDriveTrain.addVisionMeasurement(pose.pose(), Utils.fpgaToCurrentTime(pose.timestamp()));
        }
    }

}
