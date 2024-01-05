package frc4388.utility;

// This is a seperate class in case I want to encode rotation or other
// information about the tag
public class AprilTag {
    public final double x, y, z;

    public AprilTag(double _x, double _y, double _z) {
        x = _x;
        y = _y;
        z = _z;
    }
}
