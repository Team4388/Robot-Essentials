package frc4388.utility.compute;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldPositions {
    // public static final Translation2d RED_HUB_POSITION = new Translation2d(0, 0);
    // public static final Translation2d BLUE_HUB_POSITION = new Translation2d(0, 0);
    public static final Translation2d RED_HUB_POSITION = new Translation2d(11.9014494, 4.0213534);// + 0.3);
    public static final Translation2d BLUE_HUB_POSITION = new Translation2d(4.61155415, 4.0213534);// + 0.3);


    // We set the default position to one just in case it doesn't update
    // Setting to null could cause a null pointer, and setting to (0,0) could cause problems
    // I would rather have the 50/50 chance than a code error
    public static Translation2d HUB_POSITION = BLUE_HUB_POSITION;

    public static void update() {
        if(TimesNegativeOne.isRed) {
            HUB_POSITION = RED_HUB_POSITION;
        } else {
            HUB_POSITION = BLUE_HUB_POSITION;
        }
    }
}
