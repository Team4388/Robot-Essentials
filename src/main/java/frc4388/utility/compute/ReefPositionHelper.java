package frc4388.utility.compute;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc4388.robot.constants.Constants.AutoConstants;
import frc4388.robot.constants.Constants.FieldConstants;

public class ReefPositionHelper {
    public enum Side {
        LEFT,
        RIGHT,
        CENTER,
        FAR_LEFT
    }

    public static final Pose2d[] RED_TAGS = {
        FieldConstants.kTagLayout.getTagPose(6).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(7).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(8).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(9).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(10).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(11).get().toPose2d()
    };

    public static final Pose2d[] BLUE_TAGS = {
        FieldConstants.kTagLayout.getTagPose(17).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(18).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(19).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(20).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(21).get().toPose2d(),
        FieldConstants.kTagLayout.getTagPose(22).get().toPose2d()
    };

    public static double distanceTo(Pose2d first, Pose2d second){
        return Math.sqrt(Math.pow(first.getX()  -  second.getX(),2) + Math.pow(first.getY()  -  second.getY(),2));
    }


    /*
    * Function to loop through a list of tag locations to figure out closest one
    */
    public static Pose2d getNearestTag(Pose2d[] locations, Pose2d position){
        if(locations.length <= 0) return new Pose2d();

        Pose2d minPos = locations[0];
        double minDistance = distanceTo(position,minPos);

        for(int i = 1; i < locations.length; i++){
            double dist = distanceTo(locations[i],position);
            if(dist < minDistance){
                minPos = locations[i];
                minDistance = dist;
            }
        }
        
        System.out.println(minPos.getRotation().getDegrees());

        return minPos;
    }

    /*
     * Function to find closest tag location based on side
     */
    public static Pose2d getNearestTag(Pose2d position) {

        if(TimesNegativeOne.isRed)
            return getNearestTag(RED_TAGS, position);
        else
            return getNearestTag(BLUE_TAGS, position);
    }

    public static Pose2d getNearestPosition(Pose2d position, Side side, double xtrim, double ydistance) {
        return offset(getNearestTag(position), 
        getSide(side) + xtrim,
        ydistance); 
    }

    public static double getSide(Side side){
        switch(side) {
            case LEFT:
                return -(AutoConstants.X_SCORING_POSITION_OFFSET);
            case FAR_LEFT:
                return -(AutoConstants.X_SCORING_POSITION_OFFSET+Units.inchesToMeters(8));
            case RIGHT:
                return (AutoConstants.X_SCORING_POSITION_OFFSET);
            case CENTER:
                return 0;
        }
        assert false;
        return 0;
    }


    public static Pose2d offset(Pose2d oldPose, double xoffset, double yoffset){
        Translation2d oldTranslation = oldPose.getTranslation();
        
        double rot = oldPose.getRotation().getRadians();

        return new Pose2d(new Translation2d(
            oldTranslation.getX() + Math.cos(rot + Math.PI/2) * xoffset + Math.cos(rot) * yoffset,
            oldTranslation.getY() + Math.sin(rot + Math.PI/2) * xoffset + Math.sin(rot) * yoffset
        ), oldPose.getRotation().rotateBy(Rotation2d.k180deg));
    }
}
