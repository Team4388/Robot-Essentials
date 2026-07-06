package frc4388.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public final class FieldConstants {
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    
    public static final double BUMP_OFFSET = 0.4;
    public static final Transform2d BUMP_OFFSET_RED_FRONT = new Transform2d(
        Meters.of(BUMP_OFFSET),
        Meters.of(0),
        new Rotation2d()
    );
    public static final Transform2d BUMP_OFFSET_BLUE_FRONT = new Transform2d(
        Meters.of(-BUMP_OFFSET),
        Meters.of(0),
        new Rotation2d()
    );

    public static final Transform2d BUMP_OFFSET_RED_BACK = new Transform2d(
        Meters.of(-BUMP_OFFSET),
        Meters.of(0),
        new Rotation2d()
    );
    public static final Transform2d BUMP_OFFSET_BLUE_BACK = new Transform2d(
        Meters.of(BUMP_OFFSET),
        Meters.of(0),
        new Rotation2d()
    );

    // Test april tag field layout
        // public static final AprilTagFieldLayout kTagLayout = new AprilTagFieldLayout(
        //     Arrays.asList(new AprilTag[] {
        //         new AprilTag(0, new Pose3d(
        //             new Translation3d(0.,0.,0.26035), new Rotation3d(0.,0.,0.)
        //         )),
        //     }), 100, 100);

}
