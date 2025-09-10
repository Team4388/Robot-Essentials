package frc4388.utility.status;

import org.photonvision.PhotonCamera;

import frc4388.utility.status.Status.ReportLevel;

public class FaultPhotonCamera implements Queryable {
    private String name;
    private PhotonCamera cam;

    public static void addDevice(PhotonCamera cam, String name) {
        FaultPhotonCamera p = new FaultPhotonCamera();

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

        if(!cam.isConnected())
            s.addReport(ReportLevel.ERROR, "Not Connected!");

        return s;
    }
}

