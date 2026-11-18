package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;


public interface VisionIO {
    VisionFrame[] emptyFrame =
            //fix: PREVIOUS VERSION (before the fix):
            //    {new VisionFrame(false, -1, -1, new Pose3d(), 1,
            //            EstimationType.SINGLE_TARGET, -1, -1)};
            {new VisionFrame(false, -1, -1, new Pose3d(), 1, 0,
                    EstimationType.SINGLE_TARGET, -1, 0)};

    void update(VisionInputsAutoLogged inputs);

    //fix: CHANGE: new method (a default that does nothing, so the replay IO doesn't need it).
    // REASON: single-tag poses are now solved with the robot's field heading (PhotonVision's version of MegaTag2).
    // Vision gives the current fused heading to the IO every loop, before update().
    default void addHeadingData(double timestampSeconds, Rotation2d fieldHeading) {}


    //fix: CHANGE: added reprojectionError, and tagCount is now an int.
    // REASON: reprojection error is the real quality check for multi-tag (ambiguity is always 0 there).
    // A tag count can't be a fraction.
     record VisionFrame(
            boolean hasTarget,
            double timeStamp,
            double latency,
            Pose3d robotPose,
            double poseAmbiguity,
            double reprojectionError,
            EstimationType estimationType,
            double averageTargetDistance,
            //fix: PREVIOUS VERSION (before the fix):
            //    double tagCount){}
            int tagCount){}

    //fix: CHANGE: removed the max-ambiguity field from the enum.
    // REASON: ambiguity only means something for single-tag frames. Multi-tag frames are checked by
    // reprojection error instead (see Vision.checkSolveQuality).
    enum EstimationType{
        //fix: PREVIOUS VERSION (before the fix):
        //    SINGLE_TARGET(VisionConstants.maxSingleAmbiguity, VisionConstants.maxDistanceFromTargetSingle),
        //    MULTIPLE_TARGETS(VisionConstants.maxMultiAmbiguity, VisionConstants.maxDistanceFromTargetMulti);
        SINGLE_TARGET(VisionConstants.maxDistanceFromTargetSingle),
        MULTIPLE_TARGETS(VisionConstants.maxDistanceFromTargetMulti);

        //fix: PREVIOUS VERSION (before the fix):
        //    EstimationType(double maxAmbiguity, double maxDistance) {
        //        this.maxAmbiguity = maxAmbiguity;
        EstimationType(double maxDistance) {
            this.maxDistance = maxDistance;
        }

        //fix: PREVIOUS VERSION (before the fix):
        //    private final double maxAmbiguity;
        private final double maxDistance;

        //fix: PREVIOUS VERSION (before the fix):
        //    public double getMaxAmbiguity() {
        //        return maxAmbiguity;
        //    }
        public double getMaxDistance() {
            return maxDistance;
        }
    }

    @AutoLog
    class VisionInputs{
        boolean isConnected;
        VisionFrame[] visionFrames = new VisionFrame[0];
        int[] targetIDs = new int[0];
    }

}
