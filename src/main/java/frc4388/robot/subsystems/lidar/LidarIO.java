package frc4388.robot.subsystems.lidar;

import org.littletonrobotics.junction.AutoLog;

public interface LidarIO {
    @AutoLog
    public class LidarState {
        public boolean connected;
        public double distance;
    }
    
    public default void updateInputs(LidarState state) {}
}
