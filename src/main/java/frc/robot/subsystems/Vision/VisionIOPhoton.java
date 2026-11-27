package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
//fix: PREVIOUS VERSION (before the fix):
//    import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.Optional;
import java.util.function.Supplier;

//fix: CHANGE: removed @SuppressWarnings("removal") and moved to the photonlib 2026 API.
// REASON: the strategy constructor, update() and setMultiTagFallbackStrategy() are
// @Deprecated(forRemoval = true, since = "2026"). The code would break when they are deleted, and the new
// per-strategy methods let us use the heading-aided single-tag solve.
//fix: PREVIOUS VERSION (before the fix):
//    @SuppressWarnings("removal")
public class VisionIOPhoton implements VisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    //fix: CHANGE: removed referencePoseSupplier.
    // REASON: it was only used by the CLOSEST_TO_REFERENCE_POSE strategy, which was never selected.
    // The heading now comes in through addHeadingData().
    //fix: PREVIOUS VERSION (before the fix):
    //    private final Supplier<Pose2d> referencePoseSupplier;
    private Rotation2d lastHeading = null;

    //fix: PREVIOUS VERSION (before the fix):
    //    public VisionIOPhoton(CameraConstants cameraConstants, AprilTagFieldLayout fieldLayout, Supplier<Pose2d> referencePoseSupplier) {
    public VisionIOPhoton(CameraConstants cameraConstants, AprilTagFieldLayout fieldLayout) {
        camera = new PhotonCamera(cameraConstants.cameraName);

        //fix: PREVIOUS VERSION (before the fix):
        //    poseEstimator = new PhotonPoseEstimator(fieldLayout, VisionConstants.MAIN_STRATEGY, cameraConstants.robotToCamera);
        poseEstimator = new PhotonPoseEstimator(fieldLayout, cameraConstants.robotToCamera);
    }

    /**
     * //fix: CHANGE: new.
     * REASON: estimatePnpDistanceTrigSolvePose() looks up the heading at the frame's timestamp, so the
     * heading has to be added every loop. If the heading jumps (a pose reset), the buffer is cleared so frames
     * taken before the reset are never solved with a mix of the old and new heading.
     */
    //fix: PREVIOUS VERSION (before the fix):
    //    poseEstimator.setMultiTagFallbackStrategy(VisionConstants.FALLBACK_STRATEGY);
    //
    //    this.referencePoseSupplier = referencePoseSupplier;
    @Override
    public void addHeadingData(double timestampSeconds, Rotation2d fieldHeading) {
        if (lastHeading != null
                && Math.abs(fieldHeading.minus(lastHeading).getDegrees()) > VisionConstants.headingResetJumpDeg) {
            poseEstimator.resetHeadingData(timestampSeconds, fieldHeading);
        } else {
            poseEstimator.addHeadingData(timestampSeconds, fieldHeading);
        }
        lastHeading = fieldHeading;
    }


    @Override
    public void update(VisionInputsAutoLogged inputs) {
        inputs.isConnected = camera.isConnected();

        if(!inputs.isConnected){
            inputs.visionFrames = emptyFrame;
            //fix: CHANGE: clear the target IDs when disconnected.
            // REASON: otherwise the last IDs seen stayed in the log forever.
            inputs.targetIDs = new int[0];
            return;
        }

        var results = camera.getAllUnreadResults();

        inputs.visionFrames = new VisionFrame[results.size()];

        //fix: CHANGE: collect the IDs from ALL results this loop (no duplicates).
        // REASON: the old code replaced inputs.targetIDs inside the loop, so only the last result's IDs were logged.
        LinkedHashSet<Integer> targetIDs = new LinkedHashSet<>();

        for(int i = 0; i < results.size(); i++){
            inputs.visionFrames[i] = generateFrame(results.get(i));
            //fix: PREVIOUS VERSION (before the fix):
            //    ArrayList<Integer> targetIDs = new ArrayList<>();

            for(PhotonTrackedTarget target : results.get(i).getTargets()){
                targetIDs.add(target.getFiducialId());
            }
            //fix: PREVIOUS VERSION (before the fix):
            //    inputs.targetIDs = targetIDs.stream().mapToInt(Integer::intValue).toArray();
        }

        inputs.targetIDs = targetIDs.stream().mapToInt(Integer::intValue).toArray();
    }

    /**
     * generates a VisionFrame from a PhotonPipelineResult
     * @param result the result to generate the frame from
     * @return a VisionFrame instance based on the result
     */
    private VisionFrame generateFrame(PhotonPipelineResult result){
        if(!result.hasTargets()){
            return emptyFrame[0];
        }

        //fix: CHANGE: multi-tag on the coprocessor first. If there is none, the heading-aided single-tag trig solve.
        // REASON: the old fallback (LOWEST_AMBIGUITY) solves rotation and position from one tag, which can
        // "flip" to a mirrored pose. The trig solve uses the gyro heading plus the tag's angle and distance,
        // which is what 6328 (2025) and Limelight MegaTag2 do, and it has no flip.
        //fix: PREVIOUS VERSION (before the fix):
        //    poseEstimator.setReferencePose(referencePoseSupplier.get());
        Optional<EstimatedRobotPose> multiTagPose = poseEstimator.estimateCoprocMultiTagPose(result);

        //fix: PREVIOUS VERSION (before the fix):
        //    Optional<EstimatedRobotPose> poseEstimatorResult = poseEstimator.update(result);
        if (multiTagPose.isPresent()) {
            var multiTag = result.getMultiTagResult().get();

            //fix: CHANGE: average distance only over the tags used in the multi-tag solve.
            // REASON: before, every visible target was averaged, including tags not used in the solve.
            //fix: PREVIOUS VERSION (before the fix):
            //    if(poseEstimatorResult.isEmpty()){
            double distanceSum = 0;
            int distanceCount = 0;
            for (PhotonTrackedTarget target : result.getTargets()) {
                if (multiTag.fiducialIDsUsed.contains((short) target.getFiducialId())) {
                    distanceSum += target.getBestCameraToTarget().getTranslation().getNorm();
                    distanceCount++;
                }
            }
            int tagCount = multiTag.fiducialIDsUsed.size();
            double averageDistance = distanceCount > 0 ? distanceSum / distanceCount : Double.POSITIVE_INFINITY;

            return new VisionFrame(
                    true,
                    result.getTimestampSeconds(),
                    RobotController.getMeasureTime().in(Units.Seconds) - result.getTimestampSeconds(),
                    multiTagPose.get().estimatedPose,
                    multiTag.estimatedPose.ambiguity, // always 0 for multi-tag, only logged
                    multiTag.estimatedPose.bestReprojErr,
                    EstimationType.MULTIPLE_TARGETS,
                    averageDistance,
                    tagCount
            );
        }

        Optional<EstimatedRobotPose> singleTagPose = poseEstimator.estimatePnpDistanceTrigSolvePose(result);

        if (singleTagPose.isEmpty()) {
            // no heading data yet, or the tag is not in the field layout
            return emptyFrame[0];
        }

        //fix: CHANGE: ambiguity and distance now come from getBestTarget().
        // REASON: the trig solve uses getBestTarget(). The old code read targets.get(0), which could be a
        // different tag from the one used for the pose, and then divided its distance by the total target count.
        //fix: PREVIOUS VERSION (before the fix):
        //    EstimatedRobotPose robotPose = poseEstimatorResult.get();
        //
        //    EstimationType estimationType;
        //    double poseAmbiguity;
        //    double averageTargetDist = 0;
        //    if(result.multitagResult.isPresent()){
        //        estimationType = EstimationType.MULTIPLE_TARGETS;
        //        poseAmbiguity = result.multitagResult.get().estimatedPose.ambiguity;
        //
        //        for(PhotonTrackedTarget target : result.getTargets()){
        //            double distanceToTag = target.getBestCameraToTarget().getTranslation().getNorm();
        //            averageTargetDist += distanceToTag;
        //        }
        //    }
        //    else{
        //        estimationType = EstimationType.SINGLE_TARGET;
        //        poseAmbiguity = result.getTargets().get(0).poseAmbiguity;
        //
        //        averageTargetDist = result.getTargets().get(0).getBestCameraToTarget().getTranslation().getNorm();
        //    }
        //fix: END PREVIOUS VERSION
        PhotonTrackedTarget bestTarget = result.getBestTarget();

        return new VisionFrame(
                true,
                result.getTimestampSeconds(),
                RobotController.getMeasureTime().in(Units.Seconds) - result.getTimestampSeconds(),
                //fix: PREVIOUS VERSION (before the fix):
                //    robotPose.estimatedPose,
                //    poseAmbiguity,
                //    estimationType,
                //    averageTargetDist / result.getTargets().size(),
                //    result.getTargets().size()
                //fix: END PREVIOUS VERSION
                singleTagPose.get().estimatedPose,
                bestTarget.getPoseAmbiguity(),
                0,
                EstimationType.SINGLE_TARGET,
                bestTarget.getBestCameraToTarget().getTranslation().getNorm(),
                1
        );
    }
}
