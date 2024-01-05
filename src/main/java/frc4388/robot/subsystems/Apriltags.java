package frc4388.robot.subsystems;

//import edu.wpi.first.apriltag.AprilTag;
//import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Apriltags {
    public static class Tag {
        public boolean visible = true;
        public double x,  y,  z  = 0;
        public double ry, rp, rr = 0;
    }

    public Tag getTagPosRot() {
        final var tagTable = NetworkTableInstance.getDefault().getTable("apriltag");

        final Tag tag = new Tag();
        tag.visible = isAprilTag();
        tag.x  = tagTable.getEntry("TagPosX").getDouble(0);
        tag.y  = tagTable.getEntry("TagPosY").getDouble(0);
        tag.z  = tagTable.getEntry("TagPosZ").getDouble(0);
        tag.ry = tagTable.getEntry("TagRotY").getDouble(0);
        tag.rp = tagTable.getEntry("TagRotP").getDouble(0);
        tag.rr = tagTable.getEntry("TagRotR").getDouble(0);

        return tag;
    }

    public boolean isAprilTag() {
        final var tagTable = NetworkTableInstance.getDefault().getTable("apriltag");
        return tagTable.getEntry("IsTag").getBoolean(false);
    }
}
