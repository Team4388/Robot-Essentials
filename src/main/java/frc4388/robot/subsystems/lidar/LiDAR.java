package frc4388.robot.subsystems.lidar;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc4388.robot.constants.Constants.LiDARConstants;
import frc4388.utility.status.Status;
import frc4388.utility.status.FaultReporter;
import frc4388.utility.status.Queryable;
import frc4388.utility.status.Status.ReportLevel;

public class LiDAR extends SubsystemBase implements Queryable {
    LidarIO io;
    LidarStateAutoLogged state = new LidarStateAutoLogged();

    private String name = "Lidar";

    public LiDAR(LidarIO device, String name) {
        FaultReporter.register(this);

        this.io = device;
        this.name = name;
    }

    @Override
    public void periodic() {
        io.updateInputs(state);
        Logger.processInputs("LiDAR/"+name, state);
    }

    // @AutoLogOutput(key = "Lidar/{name}")
    public double getDistance(){
        return state.distance;
    }

    public boolean withinDistance(){
        if(state.distance == -1) return false;
        return state.distance < LiDARConstants.LIDAR_DETECT_DISTANCE;
    }

    @Override
    public String getName() {
        return "Lidar " + name;
    }

    @Override
    public Status diagnosticStatus() {
        Status s = new Status();

        if(state.distance == -1)
            s.addReport(ReportLevel.ERROR, "LiDAR DISCONNECTED");
        
        s.addReport(ReportLevel.INFO, "LiDAR Distance: " + state.distance);

        return s;
    }
    
}
