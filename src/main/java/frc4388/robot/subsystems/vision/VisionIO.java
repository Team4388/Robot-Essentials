package frc4388.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public class VisionState {
        public boolean isTagDetected = false;
        public boolean isTagProcessed = false;
        // public double latency = 0;
        public PoseObservation lastEstimatedPose = null;
    }

    public static record PoseObservation(
        Pose2d pose,
        double timestamp
    ) {}

    public default void updateInputs(VisionState state) {}
}
