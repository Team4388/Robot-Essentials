package frc4388.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
    private final NetworkTableEntry m_isTags;
    private final NetworkTableEntry m_xPoses;
    private final NetworkTableEntry m_yPoses;
    private final NetworkTableEntry m_zPoses;

    public Vision() {
        final var tagTable = NetworkTableInstance.getDefault().getTable("apriltag");

        m_isTags = tagTable.getEntry("IsTag");
        m_xPoses = tagTable.getEntry("TagPosX");
        m_yPoses = tagTable.getEntry("TagPosY");
        m_zPoses = tagTable.getEntry("TagPosZ");
    }

    public AprilTag[] getAprilTags() {
        if (!m_isTags.getBoolean(false)) return new AprilTag[0];

        double xarr[] = m_xPoses.getDoubleArray(new double[] {});
        double yarr[] = m_yPoses.getDoubleArray(new double[] {});
        double zarr[] = m_zPoses.getDoubleArray(new double[] {});

        AprilTag tags[] = new AprilTag[xarr.length];
        for (int i = 0; i < tags.length; i++) {
            tags[i] = new AprilTag(0, new Pose3d(xarr[i], yarr[i], zarr[i], new Rotation3d()));
        }

        return tags;
    }
}
