package frc4388.utility.status;

import frc4388.robot.subsystems.vision.RPLidarA1;
import frc4388.robot.subsystems.vision.RPLidarA1.ConnectionStatus;
import frc4388.utility.status.Status.ReportLevel;

// Fault reporter for the RPLidar A1M8 Revolving lidar sensor
public class FaultA1M8 implements Queryable {
    private String name;
    private RPLidarA1 cam;

    public static void addDevice(RPLidarA1 cam, String name) {
        FaultA1M8 p = new FaultA1M8();

        p.name = name;
        p.cam = cam;

        FaultReporter.register(p);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public Status diagnosticStatus() {
        Status s = new Status();

        ConnectionStatus cam_ConnectionStatus = cam.getStatus();

        if(cam_ConnectionStatus != ConnectionStatus.RECEIVING_DATA)
            s.addReport(ReportLevel.ERROR, "Not Connected! Status = " + cam_ConnectionStatus);

        s.addReport(ReportLevel.INFO, cam.getStatus().toString());

        return s;
    }
}
