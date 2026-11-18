// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
//fix: PREVIOUS VERSION (before the fix):
//    import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;


/*
 * //fix: ===== WHY THIS CLASS WAS CHANGED (summary) =====
 * Bug: "if vision is wrong once, it is wrong the whole match".
 * Cause: whenever a frame had 3+ tags, vision called resetOdometryCallback -> DriveBase.reset(), which ALSO
 * overwrote the Pigeon gyro with the heading vision had just calculated. Every measurement used a heading
 * std dev of 999, so nothing could ever fix the heading afterwards, and field-relative driving reads the gyro.
 * Fix (what 6328 / 254 / 2910 / 1678 do): vision never resets anything. It only feeds the Kalman filter with
 * addVisionMeasurement. Multi-tag frames get a real heading std dev, so a wrong heading heals over a few
 * seconds. Single-tag frames are solved with the fused heading and give no heading of their own.
 */
public class Vision extends SubsystemBase {
    private final HashMap<VisionCamera, Alert> disconnectedAlerts = new HashMap<>();
    //fix: PREVIOUS VERSION (before the fix):
    //    private final Consumer<Pose2d> resetOdometryCallback;

    private final VisionCamera[] cameras;

    private final VisionConsumer poseConsumer;

    //fix: CHANGE: removed resetOdometryCallback and lastOdometryResetTime.
    // REASON: this was the root cause (see the summary above). The pose is now only reset at known moments:
    // the auto start pose (PathPlanner) and the driver's reset-heading button.

    //fix: CHANGE: new inputs: the fused pose (for its heading), the yaw rate and the tilt.
    // REASON: the heading is needed for the single-tag trig solve. The yaw rate and tilt are used to throw out
    // frames taken while spinning fast or while tilted on the bump.
    //fix: PREVIOUS VERSION (before the fix):
    //    private double lastOdometryResetTime = -1.0;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final DoubleSupplier yawRateDegPerSecSupplier;
    private final DoubleSupplier tiltDegSupplier;
    //fix: CHANGE: new (Orbit 1690 idea). Seconds since DriveBase last detected a collision or a wheel slip.
    // REASON: right after one, the wheels are wrong, so vision is trusted more for a short time (see getStdDevs).
    private final DoubleSupplier secondsSinceOdometryUntrustedSupplier;

    //fix: CHANGE: new (Orbit 1690 idea). The capture time of the last ACCEPTED multi-tag frame, and the result of the
    // "is the pose confident" check (see updatePoseConfidence()).
    private double lastGoodMultiTagTime = Double.NEGATIVE_INFINITY;
    private boolean poseConfident = false;

    /**
     * Creates a new Vision.
     *
     * @param poseConsumer             where accepted measurements go (DriveBase::addVisionMeasurement)
     * @param robotPoseSupplier        the fused robot pose (DriveBase::getPose), its heading is used for single-tag solves
     * @param yawRateDegPerSecSupplier the robot yaw rate in deg/s (sign doesn't matter)
     * @param tiltDegSupplier          the robot tilt in degrees, max of |pitch| and |roll|
     * @param secondsSinceOdometryUntrustedSupplier seconds since the last collision / wheel slip (DriveBase::getSecondsSinceOdometryUntrusted)
     */
    //fix: PREVIOUS VERSION (before the fix):
    //    public Vision(VisionConsumer poseConsumer, Supplier<Pose2d> referencePoseSupplier, Consumer<Pose2d> resetOdometryCallback) {
    public Vision(VisionConsumer poseConsumer, Supplier<Pose2d> robotPoseSupplier,
                  DoubleSupplier yawRateDegPerSecSupplier, DoubleSupplier tiltDegSupplier,
                  DoubleSupplier secondsSinceOdometryUntrustedSupplier) {

        int numOfCameras = CameraConstants.values().length;

        cameras = new VisionCamera[numOfCameras];

        CameraConstants[] constants = CameraConstants.values();

        for (int i = 0; i < numOfCameras; i++) {
            //fix: PREVIOUS VERSION (before the fix):
            //    if(Constants.ROBOT_TYPE != RobotType.REPLAY) cameras[i] = new VisionCamera(constants[i], VisionConstants.FIELD_LAYOUT, referencePoseSupplier);
            //    else cameras[i] = new VisionCamera(constants[i].cameraName);
            if(Constants.ROBOT_TYPE != RobotType.REPLAY) cameras[i] = new VisionCamera(constants[i], VisionConstants.FIELD_LAYOUT);
            //fix: CHANGE: replay cameras now get their own constants.
            // REASON: the replay constructor hard-coded FRONT_CAMERA, so in replay every camera used the front
            // camera's std-dev factors, and replays did not match the real robot.
            else cameras[i] = new VisionCamera(constants[i]);

            disconnectedAlerts.put(cameras[i], new Alert(cameras[i].cameraName + " Disconnected", AlertType.kWarning));
        }
        //fix: PREVIOUS VERSION (before the fix):
        //    this.resetOdometryCallback = resetOdometryCallback;

        this.poseConsumer = poseConsumer;
        this.robotPoseSupplier = robotPoseSupplier;
        this.yawRateDegPerSecSupplier = yawRateDegPerSecSupplier;
        this.tiltDegSupplier = tiltDegSupplier;
        this.secondsSinceOdometryUntrustedSupplier = secondsSinceOdometryUntrustedSupplier; //fix: CHANGE: new
    }

    @Override
    public void periodic() {
        ArrayList<Pose3d> allAcceptedPoses = new ArrayList<>();
        ArrayList<Pose3d> allRejectedPoses = new ArrayList<>();

        //fix: CHANGE: new. Give the current fused heading to every camera before reading results.
        // REASON: the trig solve looks up the heading at each frame's timestamp, so the buffer must be up to date.
        double now = Timer.getFPGATimestamp();
        var heading = robotPoseSupplier.get().getRotation();

        //fix: CHANGE: new. Read the yaw rate and tilt once per loop.
        double yawRate = Math.abs(yawRateDegPerSecSupplier.getAsDouble());
        double tilt = Math.abs(tiltDegSupplier.getAsDouble());
        Logger.recordOutput("Vision/Yaw Rate deg per sec", yawRate);
        Logger.recordOutput("Vision/Tilt deg", tilt);

        //fix: CHANGE: new. True for a short time after a collision / wheel slip -> vision is trusted more (see getStdDevs).
        boolean odometryUntrusted =
                secondsSinceOdometryUntrustedSupplier.getAsDouble() < VisionConstants.odometryUntrustedTimeSec;
        Logger.recordOutput("Vision/Odometry Untrusted", odometryUntrusted);

        for (VisionCamera camera : cameras) {
            camera.camera.addHeadingData(now, heading);
            camera.update();
            camera.log();

            SmartDashboard.putBoolean(camera.cameraName + " is connected", camera.inputs.isConnected);
            disconnectedAlerts.get(camera).set(!camera.inputs.isConnected);

            // For logging purposes
            ArrayList<Pose3d> acceptedPoses = new ArrayList<>();
            ArrayList<Pose3d> rejectedPoses = new ArrayList<>();
            ArrayList<Translation3d> targetLocations = new ArrayList<>();
            //fix: PREVIOUS VERSION (before the fix):
            //    String rejectionReason = "";
            ArrayList<String> rejectionReasons = new ArrayList<>();

            for(int i = 0; i < camera.inputs.targetIDs.length; i++){
                Optional<Pose3d> targetPose = VisionConstants.FIELD_LAYOUT.getTagPose(camera.inputs.targetIDs[i]);

                targetPose.ifPresent(pose3d -> targetLocations.add(pose3d.getTranslation()));
            }

            Logger.recordOutput("Vision/" + camera.cameraName + "/Target Locations", targetLocations.toArray(new Translation3d[0]));

            for (VisionIO.VisionFrame frame : camera.inputs.visionFrames) {
                //fix: PREVIOUS VERSION (before the fix):
                //    if (!frame.hasTarget()) continue;
                if (!frame.hasTarget() || frame.tagCount() <= 0) continue;

                //fix: CHANGE: the rejection reason is now really recorded (one per rejected frame).
                // REASON: the old code called rejectionReason.concat(...) and threw the result away (Java strings
                // can't be changed in place), so the logged reason was always "".
                //fix: PREVIOUS VERSION (before the fix):
                //    if (!checkPoseLocation(frame.robotPose())) {
                String reason = getRejectionReason(frame, yawRate, tilt);

                if (reason != null) {
                    rejectedPoses.add(frame.robotPose());
                    //fix: PREVIOUS VERSION (before the fix):
                    //    if (!rejectionReason.contains("Invalid Location")) rejectionReason.concat("Invalid Location + ");
                    rejectionReasons.add(reason);
                    continue;
                }

                //fix: CHANGE: removed the "tagCount > 2 -> resetOdometryCallback" block that used to be here.
                // REASON: this hard reset overwrote the gyro and caused the "wrong for the whole match" bug.
                //fix: PREVIOUS VERSION (before the fix):
                //    if (!checkPoseAmbiguity(frame.poseAmbiguity(), frame.estimationType())) {
                //        rejectedPoses.add(frame.robotPose());
                //        if (!rejectionReason.contains("High Ambiguity")) rejectionReason.concat("High Ambiguity + ");
                //        continue;
                //    }
                //
                //    if (!checkDistance(frame.averageTargetDistance(), frame.estimationType())) {
                //        rejectedPoses.add(frame.robotPose());
                //        if (!rejectionReason.contains("Distance from Tag")) rejectionReason.concat("Distance from Tag");
                //        continue;
                //    }
                //    if ( frame.tagCount() > 2 && frame.timeStamp() - lastOdometryResetTime > 2.5 ) {
                //        lastOdometryResetTime = frame.timeStamp();
                //        resetOdometryCallback.accept(frame.robotPose().toPose2d());
                //    }
                //fix: END PREVIOUS VERSION

                //fix: PREVIOUS VERSION (before the fix):
                //    var stdDevs = getStdDevs(frame.averageTargetDistance(), frame.tagCount(), frame.estimationType(), camera);
                var stdDevs = getStdDevs(frame.averageTargetDistance(), frame.tagCount(), frame.estimationType(), camera,
                        odometryUntrusted);

                poseConsumer.accept(frame.robotPose().toPose2d(), frame.timeStamp(), stdDevs);

                acceptedPoses.add(frame.robotPose());
                //fix: CHANGE: new. Remember when the last good multi-tag frame was taken (used by updatePoseConfidence()).
                if (frame.estimationType() == VisionIO.EstimationType.MULTIPLE_TARGETS && frame.tagCount() >= 2)
                    lastGoodMultiTagTime = Math.max(lastGoodMultiTagTime, frame.timeStamp());
                //fix: CHANGE: new logs, so std devs and solve quality can be tuned from match logs.
                Logger.recordOutput("Vision/" + camera.cameraName + "/Last Std Devs", stdDevs.getData());
                Logger.recordOutput("Vision/" + camera.cameraName + "/Last Reprojection Error", frame.reprojectionError());
            }

            allAcceptedPoses.addAll(acceptedPoses);
            allRejectedPoses.addAll(rejectedPoses);

            Logger.recordOutput("Vision/" + camera.cameraName + "/Accepted Poses", acceptedPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/" + camera.cameraName + "/Rejected Poses", rejectedPoses.toArray(new Pose3d[0]));
            //fix: PREVIOUS VERSION (before the fix):
            //    Logger.recordOutput("Vision/" + camera.cameraName + "/Rejected Reason", rejectionReason);
            Logger.recordOutput("Vision/" + camera.cameraName + "/Rejected Reason", rejectionReasons.toArray(new String[0]));
        }
        Logger.recordOutput("Vision/All Accepted", allAcceptedPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/All Rejected", allRejectedPoses.toArray(new Pose3d[0]));

        updatePoseConfidence(now); //fix: CHANGE: new
    }

    /**
     * //fix: CHANGE: new (Orbit 1690 idea). Decides if we can trust the pose enough to shoot by it.
     * Confident = a good multi-tag frame was accepted in the last poseConfidentTimeSec seconds,
     * AND no collision / wheel slip happened after that frame (a hit after the frame makes the pose wrong again).
     * REASON: needs memory over time (when was the last good frame, was there a hit since), so it lives here
     * and not as a plain condition in RobotContainer.
     */
    private void updatePoseConfidence(double now) {
        double lastOdometryUntrustedTime = now - secondsSinceOdometryUntrustedSupplier.getAsDouble();
        double secondsSinceGoodMultiTag = now - lastGoodMultiTagTime;

        poseConfident = secondsSinceGoodMultiTag < VisionConstants.poseConfidentTimeSec
                && lastGoodMultiTagTime > lastOdometryUntrustedTime;

        Logger.recordOutput("Vision/Pose Confident", poseConfident);
        Logger.recordOutput("Vision/Seconds Since Good Multi-Tag", Math.min(secondsSinceGoodMultiTag, 999));
    }

    /**
     * //fix: CHANGE: new. Used by RobotContainer.readyToShoot() (teleop only) and shown on Elastic.
     *
     * @return true if the pose is good enough to aim and shoot by it
     */
    public boolean isPoseConfident() {
        return poseConfident;
    }

    /**
     * //fix: CHANGE: new. All the rejection checks in one place, in order from cheapest to most expensive.
     * Note: there is deliberately NO "too far from the current pose" check. That kind of check is what locks
     * a robot out for good once its pose is wrong. 254 / 6328 scale the std devs instead.
     *
     * @return null if the frame is good, otherwise a short reason for the log
     */
    private String getRejectionReason(VisionIO.VisionFrame frame, double yawRateDegPerSec, double tiltDeg) {
        if (frame.estimationType() == null) return "No Estimation Type";

        // REASON: motion blur / timestamp error while spinning fast (Limelight MegaTag2 rule)
        if (yawRateDegPerSec > VisionConstants.maxYawRateDegPerSec) return "Spinning Too Fast";

        // REASON: robotToCamera assumes the robot is flat. On the bump the camera is tilted and the pose is shifted.
        if (tiltDeg > VisionConstants.maxTiltDeg) return "Robot Tilted";

        if (!checkPoseLocation(frame.robotPose(), frame.estimationType())) return "Invalid Location";

        if (!checkSolveQuality(frame)) return frame.estimationType() == VisionIO.EstimationType.MULTIPLE_TARGETS
                ? "High Reprojection Error" : "High Ambiguity";

        if (!checkDistance(frame.averageTargetDistance(), frame.estimationType())) return "Distance from Tag";

        return null;
    }

    /**
     * If the pose is in a valid location.
     * The pose is valid if it is within the field
     * and the height of the robot is in tolerance
     *
     * @param pose the calculated pose
     * @param estimationType the type of estimation used
     * @return whether the pose is in a valid location
     */
    //fix: PREVIOUS VERSION (before the fix):
    //    private boolean checkPoseLocation(Pose3d pose) {
    private boolean checkPoseLocation(Pose3d pose, VisionIO.EstimationType estimationType) {
        double poseX = pose.getX();
        double poseY = pose.getY();
        double poseZ = pose.getZ();

        if (poseX >= VisionConstants.FIELD_LAYOUT.getFieldLength() || poseX < 0) return false;
        if (poseY >= VisionConstants.FIELD_LAYOUT.getFieldWidth() || poseY < 0) return false;
        //fix: PREVIOUS VERSION (before the fix):
        //    return !(Math.abs(poseZ) > VisionConstants.maxHeightDeviation);

        //fix: CHANGE: the height check only runs for multi-tag.
        // REASON: the single-tag trig solve always returns z = 0, so checking it means nothing.
        if (estimationType == VisionIO.EstimationType.MULTIPLE_TARGETS)
            return !(Math.abs(poseZ) > VisionConstants.maxHeightDeviation);
        return true;
    }

    /**
     * //fix: CHANGE: replaces checkPoseAmbiguity().
     * REASON: for multi-tag, PhotonVision's ambiguity is always 0, so that check never rejected anything.
     * Multi-tag is now checked by reprojection error (pixels). Single-tag is still checked by ambiguity.
     *
     * @param frame the frame to check
     * @return whether the solve quality is within acceptable bounds
     */
    //fix: PREVIOUS VERSION (before the fix):
    //    private boolean checkPoseAmbiguity(double poseAmbiguity, VisionIO.EstimationType estimationType) {
    private boolean checkSolveQuality(VisionIO.VisionFrame frame) {
        if (frame.estimationType() == VisionIO.EstimationType.MULTIPLE_TARGETS) {
            return frame.reprojectionError() <= VisionConstants.maxMultiReprojectionError;
        }
        return frame.poseAmbiguity() <= VisionConstants.maxSingleAmbiguity;
    }

    private boolean checkDistance(double distance, VisionIO.EstimationType estimationType){
        if (estimationType == null) {
            return false;
        }
        //fix: PREVIOUS VERSION (before the fix):
        //    return poseAmbiguity <= estimationType.getMaxAmbiguity();
        return distance <= estimationType.getMaxDistance();
    }

    /**
    //fix: PREVIOUS VERSION (before the fix):
    //    private boolean checkDistance(double distanceX, VisionIO.EstimationType estimationType){
    //        if (estimationType == null) {
    //            return false;
    //        }
    //        return distanceX <= estimationType.getMaxDistance();
    //    }
    //
    //        /**
    //fix: END PREVIOUS VERSION
     * calculates the standard deviations for the pose based on the average distance to the tags
     * (same shape as the AdvantageKit / 6328 template: factor * distance^2 / tagCount)
     *
     * @param averageTagDistance the average distance to the tags
     * @param tagCount           the number of tags used in the solve
     * @param estimationType     the type of estimation used
     * @param camera             the camera (for its per-camera factors)
     * @param odometryUntrusted  true right after a collision / wheel slip (x/y is then trusted more)
     * @return the standard deviations for the pose
     */
    //fix: PREVIOUS VERSION (before the fix):
    //    private Matrix<N3, N1> getStdDevs(double averageTagDistance, double tagCount, VisionIO.EstimationType estimationType, VisionCamera camera) {
    //fix: PREVIOUS VERSION (after the first vision fix, before the collision/slip change):
    //    private Matrix<N3, N1> getStdDevs(double averageTagDistance, int tagCount, VisionIO.EstimationType estimationType, VisionCamera camera) {
    private Matrix<N3, N1> getStdDevs(double averageTagDistance, int tagCount, VisionIO.EstimationType estimationType,
                                      VisionCamera camera, boolean odometryUntrusted) {
        // Calculate standard deviations
        double stdDevFactor =
                Math.pow(averageTagDistance, 2.0) / tagCount;

        double linearStdDev = camera.constants.XYstdFactor * stdDevFactor;
        double angularStdDev = camera.constants.thetaStdFactor * stdDevFactor;
        if (estimationType == VisionIO.EstimationType.MULTIPLE_TARGETS) {
            //fix: PREVIOUS VERSION (before the fix):
            //    linearStdDev *= 0.5;
            //    angularStdDev *= 0.5;
            linearStdDev *= VisionConstants.multiTagStdDevFactor;
            angularStdDev *= VisionConstants.multiTagStdDevFactor;
        } else {
            //fix: CHANGE: single-tag frames give NO heading (infinite std dev).
            // REASON: the single-tag trig solve returns the heading WE gave it, so feeding it back as a
            // "measurement" would just be the robot agreeing with itself. AdvantageKit (MegaTag2),
            // Limelight (9999999) and 1678 do the same.
            angularStdDev = Double.POSITIVE_INFINITY;
        }

        //fix: CHANGE: new (Orbit 1690 idea). Right after a collision / wheel slip, trust vision's x/y more.
        // REASON: the wheels just "lied", so vision should fix the position fast instead of over a few seconds.
        // The heading is not changed (the gyro still measures turning correctly during a hit).
        if (odometryUntrusted) linearStdDev *= VisionConstants.odometryUntrustedStdDevFactor;

        return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
    }



    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d pose, double timeStamp, Matrix<N3, N1> stdDevs);
    }


    private static class VisionCamera {
        private final VisionIO camera;
        private final String cameraName;
        private final VisionInputsAutoLogged inputs;
        private final CameraConstants constants;

        public VisionCamera(VisionIO camera, String cameraName, CameraConstants constants) {
            this.camera = camera;
            this.cameraName = cameraName;
            this.inputs = new VisionInputsAutoLogged();
            this.constants = constants;
        }

        //fix: PREVIOUS VERSION (before the fix):
        //    public VisionCamera(CameraConstants constants, AprilTagFieldLayout fieldLayout, Supplier<Pose2d> referencePoseSupplier) {
        //        this(new VisionIOPhoton(constants, fieldLayout, referencePoseSupplier), constants.cameraName, constants);
        public VisionCamera(CameraConstants constants, AprilTagFieldLayout fieldLayout) {
            this(new VisionIOPhoton(constants, fieldLayout), constants.cameraName, constants);
        }

        /**
         * replay constructor (the IO does nothing, the inputs come from the log)
         * //fix: CHANGE: takes the camera's own CameraConstants instead of only a name (it used to hard-code FRONT_CAMERA).
         */
        //fix: PREVIOUS VERSION (before the fix):
        //    public VisionCamera(String cameraName){
        //        this.camera = new VisionIO() {
        public VisionCamera(CameraConstants constants){
            this(new VisionIO() {
                @Override
                public void update(VisionInputsAutoLogged inputs) {
                }
            //fix: PREVIOUS VERSION (before the fix):
            //    };
            //
            //    this.cameraName = cameraName;
            //    this.constants = CameraConstants.FRONT_CAMERA;
            //    inputs = new VisionInputsAutoLogged();
            //fix: END PREVIOUS VERSION
            }, constants.cameraName, constants);
        }

        public void update() {
            camera.update(inputs);
        }

        public void log() {
            Logger.processInputs("Camera/" + cameraName, inputs);
        }
    }
}
