package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

//fix: PREVIOUS VERSION (before the fix):
//    import org.photonvision.PhotonPoseEstimator;
public class VisionConstants {

    //fix: CHANGE: removed MAIN_STRATEGY / FALLBACK_STRATEGY.
    // REASON: the strategy-based PhotonPoseEstimator API is @Deprecated(forRemoval = true) in photonlib 2026.
    // VisionIOPhoton now calls estimateCoprocMultiTagPose() first and estimatePnpDistanceTrigSolvePose() as the fallback.
    //fix: PREVIOUS VERSION (before the fix):
    //    public static final PhotonPoseEstimator.PoseStrategy MAIN_STRATEGY =
    //            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    //fix: CHANGE: added a warning comment (the value is the same).
    // REASON: 2026 has two field layouts (Welded and AndyMark). This value, the layout uploaded to the
    // PhotonVision coprocessor (Settings > Import Settings > AprilTag Layout) and the event's field must all match.
    // Coprocessor multi-tag uses the coprocessor's copy, so a mismatch shifts every multi-tag pose.
    //fix: PREVIOUS VERSION (before the fix):
    //    public static final PhotonPoseEstimator.PoseStrategy FALLBACK_STRATEGY =
    //            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    /** max |z| of a multi-tag robot pose in meters (the robot is on the floor, so a large z means a bad solve) */
    public static final double maxHeightDeviation = 0.1;

    //fix: CHANGE: maxMultiAmbiguity was replaced by maxMultiReprojectionError.
    // REASON: PhotonVision always reports ambiguity = 0 for multi-tag (PnpResult(best, err) has no alternate
    // solution), so the old multi-tag ambiguity check never rejected anything. Reprojection error (pixels, RMS) is
    // the real quality metric for a multi-tag solve. 4 px is lenient on purpose: check "Reprojection Error"
    // in the logs and tighten it later.
    //fix: PREVIOUS VERSION (before the fix):
    //    public static final double maxMultiAmbiguity = 0.2;
    //    public static final double maxSingleAmbiguity = 0.2;
    //    public static final double maxDistanceFromTargetSingle = 2.5;
    //    public static final double maxDistanceFromTargetMulti = 2; // If there is a season where you can see one tag real close and one far fix this.
    //fix: END PREVIOUS VERSION
    public static final double maxMultiReprojectionError = 4.0;

    //fix: CHANGE: 0.2 -> 0.3 (same value as the AdvantageKit / 6328 vision template).
    // REASON: single-tag poses are now solved with the gyro heading (trig solve), so the ambiguous "flip" no
    // longer matters much. This only throws out really bad detections such as a half-covered tag.
    public static final double maxSingleAmbiguity = 0.3;

    //fix: CHANGE: single 2.5 -> 3.0, multi 2.0 -> 5.0.
    // REASON: the old limits were backwards. Multi-tag is the MORE reliable solve at range, but it was cut off at
    // 2 m, so most good multi-tag frames were thrown away. Std devs already grow with distance^2, so far
    // frames are trusted less instead of being rejected.
    public static final double maxDistanceFromTargetSingle = 3.0;
    public static final double maxDistanceFromTargetMulti = 5.0;

    //fix: CHANGE: new. Vision is ignored while spinning faster than this (deg/s).
    // REASON: motion blur + timestamp error make poses bad while spinning fast. Limelight's MegaTag2 example uses
    // 360 deg/s, 254 (2024) used 200 deg/s.
    public static final double maxYawRateDegPerSec = 360;

    //fix: CHANGE: new. Vision is ignored while the robot is tilted more than this (deg) (pitch or roll).
    // REASON: REBUILT has bumps. robotToCamera assumes the robot is flat, so when it is tilted every pose is
    // shifted, and the trig solve projects onto the floor with the wrong angle.
    public static final double maxTiltDeg = 5;

    //fix: CHANGE: new. A heading jump bigger than this between two loops is treated as a pose reset.
    // REASON: after DriveBase.reset() the heading jumps. Frames taken before the reset must not be solved with
    // a heading that is half-way between the old and new heading, so the heading buffer is cleared.
    public static final double headingResetJumpDeg = 30;

    //fix: CHANGE: new. Multiplier for multi-tag std devs (applied to x/y and theta).
    // REASON: the old code had the same 0.5 factor hard-coded inside getStdDevs(). It is now a named constant.
    public static final double multiTagStdDevFactor = 0.5;

    public enum CameraConstants{
        FRONT_CAMERA("Front Camera",
            new Transform3d(
                0.11143, 0.221, 0.52,
                //0.13,0.32,0.52,
                //0,7,16
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-8), Units.degreesToRadians(-10))),
                //fix: CHANGE: thetaSTD 999 -> 0.3.
                // REASON: with 999 vision could never correct the heading, so one wrong heading lasted the whole
                // match. Now multi-tag frames correct the heading slowly (single-tag frames still give no heading,
                // see Vision.getStdDevs). 0.3 = 1.5x the XY factor, the same ratio idea as AdvantageKit (0.02 / 0.06).
                //fix: PREVIOUS VERSION (before the fix):
                //    0.2, 999),
                0.2, 0.3),

       //fix: CHANGE: added a TODO (the values are the same).
       // REASON: the comment says this camera points BACKWARDS, but its yaw is 0 deg (pointing forward). If it really
       // points backwards the yaw must be 180 deg (Math.PI), otherwise every pose from this camera is wrong. I can't
       // measure the robot, so check it: in AdvantageScope, compare "Vision/Right Camera/Accepted Poses" against
       // "Vision/Front Camera/Accepted Poses" while both see tags.
       POINTING_OUT_CAMERA("Right Camera", //this camera will be placed on the side of the shooter, pointing backwards
           new Transform3d(
               0.23554, -0.26835, 0.47246,
               //fix: PREVIOUS VERSION (before the fix):
               //    new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(0))),
               //    0.2, 999);
               new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(0))), // TODO verify yaw (0 vs 180)
               //fix: CHANGE: thetaSTD 999 -> 0.3 (same reason as FRONT_CAMERA)
               0.2, 0.3);

        public final String cameraName;
        public final double XYstdFactor;
        public final double thetaStdFactor;

        public final Transform3d robotToCamera;

        CameraConstants(String name, Transform3d robotToCamera, double xySTD, double thetaSTD){
            this.cameraName = name;
            this.robotToCamera = robotToCamera;
            this.XYstdFactor = xySTD;
            this.thetaStdFactor = thetaSTD;
        }

        @Override
        public String toString(){
            return cameraName;
        }
    }

}
