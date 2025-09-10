package frc4388.robot.subsystems.swerve;

import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc4388.robot.subsystems.vision.VisionIO.PoseObservation;

public interface SwerveIO {
    @AutoLog
    public class SwerveState {
        public Pose2d currentPose = null;
        public Pose2d lastPose = null;
        public ChassisSpeeds speeds = null;
        public double odometryRate = 1;
    }

    public default void setControl(SwerveRequest ctrl) {}

    public default void setLimits(double limitInAmps) {}

    public default void tareEverything() {}

    public default void resetPose(Pose2d pose) {}

    public default void addVisionMeasurement(List<PoseObservation> poses) {}

    public default void updateInputs(SwerveState state) {}
}
