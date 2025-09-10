package frc4388.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import frc4388.robot.constants.Constants.FieldConstants;
import frc4388.robot.constants.Constants.VisionConstants;

public class VisionReal implements VisionIO {
    // private PhotonCamera[] cameras;
    // private PhotonPoseEstimator[] estimators;
    
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;

    // public List<EstimatedRobotPose> poses = new ArrayList<>();

    
    public VisionReal(PhotonCamera camera, Transform3d position){
        this.camera = camera;
        estimator = new PhotonPoseEstimator(FieldConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, position);
        estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // private Instant lastVisionTime = null;


    public void updateInputs(VisionState state) {
        state.isTagProcessed = false;
        state.isTagDetected = false;

        state.lastEstimatedPose = null;

        var results = camera.getAllUnreadResults();

        // If there are no more updates from the camera
        if (results.size() == 0) 
            return;

        
        var result = results.get(results.size()-1);

        state.isTagDetected = state.isTagDetected | result.hasTargets();

        // If there are no tags
        if(!result.hasTargets())
            return;

        Optional<EstimatedRobotPose> estimatedRobotPose = getEstimatedGlobalPose(result, estimator);

        // If the tag was failed to be processed
        if(estimatedRobotPose.isEmpty())
            return;
        
        EstimatedRobotPose pose = estimatedRobotPose.get();

        state.lastEstimatedPose = new PoseObservation(pose.estimatedPose.toPose2d(), pose.timestampSeconds);

        state.isTagProcessed = true;
    }


    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonPipelineResult change, PhotonPoseEstimator estimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        var targets = change.getTargets();
        for(int i = targets.size()-1; i >= 0; i--){
            Transform3d pos = targets.get(i).getBestCameraToTarget();
            double distance = Math.sqrt(Math.pow(pos.getX(),2) + Math.pow(pos.getY(),2) + Math.pow(pos.getZ(),2));
            if (distance > VisionConstants.MIN_ESTIMATION_DISTANCE) {
                change.targets.remove(i);
            }
        }

        if(targets.size() <= 0)
            return visionEst; // Will be empty

        visionEst = estimator.update(change);
        // updateEstimationStdDevs(visionEst, change.getTargets(), estimator);

        return visionEst;
    }

    public String getName() {
        return camera.getName();
    }


    // /**
    //  * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
    //  * deviations based on number of tags, estimation strategy, and distance from the tags.
    //  *
    //  * @param estimatedPose The estimated pose to guess standard deviations for.
    //  * @param targets All targets in this camera frame
    //  */
    // private void updateEstimationStdDevs(
    //         Optional<EstimatedRobotPose> estimatedPose, 
    //         List<PhotonTrackedTarget> targets,
    //         PhotonPoseEstimator estimator) {
    //     if (estimatedPose.isEmpty()) {
    //         // No pose input. Default to single-tag std devs
    //         curStdDevs = VisionConstants.kSingleTagStdDevs;

    //     } else {
    //         // Pose present. Start running Heuristic
    //         var estStdDevs = VisionConstants.kSingleTagStdDevs;
    //         int numTags = 0;
    //         double avgDist = 0;

    //         // Precalculation - see how many tags we found, and calculate an average-distance metric
    //         for (var tgt : targets) {
    //             var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
    //             if (tagPose.isEmpty()) continue;
                
    //             double distance = tagPose
    //             .get()
    //             .toPose2d()
    //             .getTranslation()
    //             .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
                
    //             if (distance < VisionConstants.MIN_ESTIMATION_DISTANCE) {
    //                 numTags++;
    //                 avgDist += distance;
    //             }
    //         }

    //         if (numTags == 0) {
    //             // No tags visible. Default to single-tag std devs
    //             curStdDevs = VisionConstants.kSingleTagStdDevs;
    //         } else {
    //             // One or more tags visible, run the full heuristic.
    //             avgDist /= numTags;
    //             // Decrease std devs if multiple targets are visible
    //             if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
    //             // Increase std devs based on (average) distance
    //             if (numTags == 1 && avgDist > 4)
    //                 estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    //             else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
    //             curStdDevs = estStdDevs;
    //         }
    //     }
    // }
}
