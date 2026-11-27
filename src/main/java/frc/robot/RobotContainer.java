
package frc.robot;

import static edu.wpi.first.units.Units.Second;

import java.util.function.Supplier;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.json.simple.parser.ParseException;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands; // FIX: needed to register the autos' named commands
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;
import java.io.IOException;
import edu.wpi.first.math.MathUtil; //fix: CHANGE: new import, used by readyToShoot()
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drive.AimDriving;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Intake.Rollers.RollerCommands;
import frc.robot.subsystems.Intake.Rollers.RollerConstants;
import frc.robot.subsystems.Intake.Rollers.Rollers;
import frc.robot.subsystems.Intake.Rollers.rollersReverse;
import frc.robot.subsystems.Intake.rackAndPinion.CycleIntake;
import frc.robot.subsystems.Intake.rackAndPinion.intakeFullExtension;
import frc.robot.subsystems.Intake.rackAndPinion.intakeFullRetraction;
import frc.robot.subsystems.Intake.rackAndPinion.rackAndPinionSubsystem;
import frc.robot.subsystems.Intake.transfer.transferOn;
import frc.robot.subsystems.Intake.transfer.transferReverse;
import frc.robot.subsystems.Intake.transfer.transferSubsystem;
import frc.robot.subsystems.Shooter.flywheel.flyWheelCommand;
import frc.robot.subsystems.Shooter.flywheel.flyWheelConstants;
import frc.robot.subsystems.Shooter.flywheel.flyWheelsubsystem;
import frc.robot.subsystems.Shooter.hood.hoodConstants;
import frc.robot.subsystems.Shooter.hood.hoodLerp;
import frc.robot.subsystems.Shooter.hood.homeHood;
import frc.robot.subsystems.Shooter.hood.hoodSubsystem;
import frc.robot.subsystems.Shooter.hood.openHood;
import frc.robot.subsystems.Shooter.kicker.kickerOn;
import frc.robot.subsystems.Shooter.kicker.kickerSubsystem;
import frc.util.HubTracker;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;

public class RobotContainer {

    public static Alert controllerDisconnected = new Alert("Drive Controller Disconnect", AlertType.kError);
    public static Alert brownoutBattery = new Alert("BROWNOUT", AlertType.kError);

    public static CommandPS5Controller driveController;
    public static CommandPS5Controller driveControllerAdam;
    public static DriveBase driveBase;
    public static hoodSubsystem hood;
    public static flyWheelsubsystem flywheel;
    public static kickerSubsystem kicker;
    public static transferSubsystem transfer;
    public static rackAndPinionSubsystem rackAndPinion;
    public static Rollers rollers;
    public static Vision vision;

    public static Runnable fuelIncrementer;
    public static int fuelApproximation = 0;

    public static PathPlannerPath upLeftTrenchPath;
    public static PathPlannerPath upRightTrenchPath;

    public static boolean intakeExtended = false;
    public static boolean isCyclingIntake = false;
    public static boolean stopCyclingRequested = false;
    public static boolean isShooting = false;

    public static LoggedDashboardChooser<Command> autoChooser;
    public static LoggedDashboardChooser<String> sideChooser;
    public static LoggedDashboardChooser<String> preWarm;
    public static HashMap<String, Command> mirroredAutoMap = new HashMap<>();

    // #region Premade Commands
    public static Supplier<Command> intakeWithSlowDrive;
    public static Supplier<Command> extendIntake;
    public static Supplier<Command> retractIntake;
    public static Supplier<Command> toggleIntake;
    public static Supplier<Command> outtake;
    public static Supplier<Command> pass;
    public static Supplier<Command> shoot;
    public static Supplier<Command> shootWhileAuto;
    public static Supplier<Command> feed;
    public static Supplier<Command> cycleIntakeWhileShooting;
    public static Supplier<Command> warmpupshooterMin;
    public static Supplier<Command> warmpupshooterAutoBump;
    public static Supplier<Command> warmpupshooterAutoTrench;
    public static Supplier<Command> aimAtHub;
    public static Supplier<Command> shootWhenReady;
    public static Supplier<Command> manualShoot; // fix: CHANGE: new, the manual shot (cross button)
    public static Supplier<Command> startRollers;
    public static Supplier<Command> stopRollers;
    public static Supplier<Command> Test;
    public static Supplier<Command> homeHood;
    public static Supplier<Command> toggleHood;
    public static Supplier<Command> extendHood;
    public static Supplier<Command> retractHood;

    public static boolean ishoodopen = false;

    // #endregion

    public RobotContainer() {
        driveController = new CommandPS5Controller(0);
        driveBase = new DriveBase();
        hood = new hoodSubsystem();
        flywheel = new flyWheelsubsystem();
        kicker = new kickerSubsystem();
        transfer = new transferSubsystem();
        rackAndPinion = new rackAndPinionSubsystem();
        rollers = new Rollers();

        // fix: CHANGE: vision no longer gets driveBase::reset. It gets yaw rate + tilt
        // instead, for its rejection filters.
        // REASON: vision calling reset() (which also reset the gyro) whenever it saw 3+
        // tags is what made one bad frame
        // ruin the whole match. Vision now only sends measurements to the pose
        // estimator.
        // fix: PREVIOUS VERSION (before the fix):
        // vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose,
        // driveBase::reset);
        // fix: CHANGE: also passes getSecondsSinceOdometryUntrusted (collision / wheel
        // slip detection, Orbit 1690 idea).
        // fix: PREVIOUS VERSION (before the fix):
        // vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose,
        // driveBase::getYawRateDegPerSec, driveBase::getTiltDeg);
        vision = new Vision(driveBase::addVisionMeasurement, driveBase::getPose, driveBase::getYawRateDegPerSec,
                driveBase::getTiltDeg,
                driveBase::getSecondsSinceOdometryUntrusted);

        configureCommands();
        configureNamedCommands();
        configureDefaultCommands();
        configureMechanismBindings();
        configureDriveBindings();
        configureChoosers();
    }
    
    // #region Commands
    public static void configureCommands() {
        fuelIncrementer = () -> fuelApproximation += 3;

        homeHood = () -> new homeHood(hood).withName("Hood Homing");
 
        intakeWithSlowDrive = () ->
            new RollerCommands(rollers)
                .alongWith(Commands.startEnd(DriveCommand::slowSpeed, DriveCommand::fullSpeed))
                .withName("Intake");
 
        extendIntake = () ->
            new intakeFullExtension(rackAndPinion)
                .beforeStarting(() -> intakeExtended = true)
                .withName("Extend Intake");
 
        retractIntake = () ->
            new intakeFullRetraction(rackAndPinion)
                .beforeStarting(() -> intakeExtended = false)
                .withName("Retract Intake");
 
        toggleIntake = () -> Commands.runOnce(() -> {
        if (intakeExtended) {
            CommandScheduler.getInstance().schedule(retractIntake.get());
        } else {
            CommandScheduler.getInstance().schedule(extendIntake.get());
        }
        });
        extendHood = () -> 
            new openHood(hood, () -> flyWheelConstants.shouldMAX)
            .beforeStarting(() -> ishoodopen = true)
            .withName("Extend hood");
        retractHood = () -> 
            new openHood(hood, () -> flyWheelConstants.shouldMIN)
            .beforeStarting(() -> ishoodopen = false)
            .withName("Retract hood");

        toggleHood = () -> Commands.runOnce(() -> {
            if (ishoodopen) {
                CommandScheduler.getInstance().schedule(retractHood.get());
            } else {
                CommandScheduler.getInstance().schedule(extendHood.get());
            }
            });
 
        outtake = () ->
            Commands.parallel(
                Commands.either(Commands.none(), extendIntake.get(), () -> intakeExtended),
                new rollersReverse(rollers),
                Commands.either(new transferReverse(transfer), Commands.none(), () -> !inAllianceZone())
            ).withName("Outtake");
        pass = () ->
            Commands.parallel(
                new StartEndCommand(() -> isShooting = true, () -> isShooting = false),
                new flyWheelCommand(flywheel, RobotContainer::passDistance),
                new openHood(hood, () -> passDistance()),
                new AimDriving(driveBase, driveController, RobotContainer::getShootingAngle)
            ).withName("Pass");
        shoot = () ->
            Commands.parallel(
                new StartEndCommand(() -> isShooting = true, () -> isShooting = false),
                new flyWheelCommand(flywheel, RobotContainer::distanceFromHub),
                new openHood(hood, () -> distanceFromHub()),
                new AimDriving(driveBase, driveController, RobotContainer::getShootingAngle)
            ).withName("Shoot");
        //fix: CHANGE: new. Manual shot that does NOT use the pose at all.
        // REASON: the normal shot takes the hood angle, the flywheel RPM and the robot angle from the pose. If the pose
        // is wrong, all three are wrong, and readyToShoot() now also refuses to feed when the pose is not confident.
        // This shot uses a fixed distance (MANUAL_SHOOTING_DISTANCE) and does NOT take the drivetrain, so the driver
        // keeps normal driving and aims by hand. It always works, even if vision is completely wrong.
        manualShoot = () ->
            Commands.parallel(
                new StartEndCommand(() -> isShooting = true, () -> isShooting = false),
                new flyWheelCommand(flywheel, () -> Constants.ConstantShootingRanges.MANUAL_SHOOTING_DISTANCE),
                new openHood(hood, () -> Constants.ConstantShootingRanges.MANUAL_SHOOTING_DISTANCE)
            ).withName("Manual Shoot");
        shootWhileAuto = () -> Commands.runOnce(() -> {
            if (areBallsGoingIntoShooter()) {
                CommandScheduler.getInstance().schedule(shoot.get());
            } 
            });

        feed = () ->
            new transferOn(transfer)
            .alongWith(new kickerOn(kicker))
            .withName("Feed");
        cycleIntakeWhileShooting = () ->
            new CycleIntake(rackAndPinion)
                .alongWith(Commands.startEnd(() -> isCyclingIntake = true, () -> isCyclingIntake = false))
                .until(() -> stopCyclingRequested)
                .beforeStarting(() -> stopCyclingRequested = false)
                .withName("cycleIntakeWhileShooting");
        warmpupshooterMin = () ->
            new flyWheelCommand(flywheel, () -> Constants.ConstantShootingRanges.MIN_SHOOTING_DISTANCE)
                .withName("WarmpupshooterMin");
        warmpupshooterAutoBump = () ->
            new flyWheelCommand(flywheel, () -> Constants.ConstantShootingRanges.AUTO_BUMP_SHOOTING_DISTANCE)
                .withName("WarmpupshooterAutoBump");
        warmpupshooterAutoTrench = () ->
            new flyWheelCommand(flywheel, () -> Constants.ConstantShootingRanges.AUTO_TRENCH_SHOOTING_DISTANCE)
                .withName("WarmpupshooterAutoTrench");
        aimAtHub = () ->
            new AimDriving(driveBase, driveController, RobotContainer::angleToHub)
                .withName("Aim At Hub");
        shootWhenReady = () ->
            shoot.get().alongWith(
                Commands.waitUntil(RobotContainer::readyToShoot).withTimeout(1.5)
                    .andThen(feed.get()))
                .withName("Shoot When Ready");
        startRollers = () ->
            Commands.runOnce(() -> rollers.setDutyCycle(RollerConstants.DUTY_CYCLE), rollers)
                .withName("Start Rollers");
        stopRollers = () ->
            Commands.runOnce(rollers::stopMotor, rollers)
                .withName("Stop Rollers");
        // hood1 = () ->
        // Commands.runOnce(() -> hood.openToPosition(2), hood)
        // .withName("hood Lerp");
        Test = () -> 

        // new openHood(hood, () -> 9999)
        // new kickerOn(kicker)
        // new transferOn(transfer)
            new flyWheelCommand(flywheel, () -> 3)
                // .alongWith(new transferOn(transfer))
                .alongWith(new kickerOn(kicker))
                // .alongWith(new RollerCommands(rollers))
                .withName("test");
        }

    public static void configureNamedCommands() {
        NamedCommands.registerCommand("shoot to hub", shootWhenReady.get().withTimeout(6));
        NamedCommands.registerCommand("aim to hub", aimAtHub.get().withTimeout(0.5));
        NamedCommands.registerCommand("open intake", extendIntake.get().withTimeout(0.6));
        NamedCommands.registerCommand("intake", intakeWithSlowDrive.get());
        NamedCommands.registerCommand("eject", outtake.get().withTimeout(1));
        NamedCommands.registerCommand("warm up shooter bump", warmpupshooterAutoBump.get());
        NamedCommands.registerCommand("warm up shooter trench", warmpupshooterAutoTrench.get());
        NamedCommands.registerCommand("start rollers", startRollers.get());
        NamedCommands.registerCommand("stop rollers", stopRollers.get());
    }

    public static void configureDefaultCommands() {
    }
    // #endregion

    // #region Bindings
    public static void configureMechanismBindings() {
        // driveController.L1().whileTrue(intakeWithSlowDrive.get());
        // driveController.L2().whileTrue(pass.get());

        // driveController.R1().whileTrue(shoot.get());

        // driveController.R1().and(RobotContainer::readyToShoot).whileTrue(feed.get()).whileTrue(cycleIntakeWhileShooting.get());

        // fix: CHANGE: new. Hold cross = manual shot (fixed distance, the driver aims).
        // Feeds when the hood and flywheel
        // are ready. It does not check the pose or the robot angle at all, so it still
        // works when the pose is wrong.
        // driveController.cross().whileTrue(manualShoot.get());
        // driveController.cross().and(RobotContainer::manualReadyToShoot).whileTrue(feed.get()).whileTrue(cycleIntakeWhileShooting.get());

        // driveController.triangle().whileTrue(outtake.get());
        // driveController.R3().onTrue(toggleHood.get());
        // driveController.square().onTrue(
        // Commands.runOnce(() -> {
        // if (isCyclingIntake) {
        // stopCyclingRequested = true;
        // }
        // else {
        // CommandScheduler.getInstance().schedule(toggleIntake.get());
        // }
        // })
        // );
        // driveController.R1().whileTrue(Flywheel.get());
        // new Trigger(() -> rackAndPinion.isJammed() && intakeExtended)
        // .onTrue(retractIntake.get());

        // new Trigger(() -> ((HubTracker.isActive(Robot.isRedAlliance ? Alliance.Red :
        // Alliance.Blue)) && !isShooting))
        // .whileTrue(warmpupshooterMin.get());
        // new Trigger(() ->(getPreWarm() && !isShooting))
        // .whileTrue(warmpupshooterMin.get());
        // driveController.R1().whileTrue(Test.get());
        // driveController.L3().whileTrue(new hoodLerp(hood, 50));

    }

    public static void configureDriveBindings() {
        // new Trigger(RobotState::isEnabled).and(() -> !hood.isHomed()).onTrue(homeHood.get());
        // new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(
        //         new StartEndCommand(() -> driveBase.setDefaultCommand(new DriveCommand(driveBase, driveController)),
        //                 driveBase::removeDefaultCommand)
        //                 .ignoringDisable(true));

        driveController.PS().onTrue(driveBase.resetOnlyDirection());

        // driveController.povRight().whileTrue(new MinorAdjust(driveBase,
        // AdjustmentDirection.RIGHT));
        // driveController.povLeft().whileTrue(new MinorAdjust(driveBase,
        // AdjustmentDirection.LEFT));
        // driveController.povUp().whileTrue(new MinorAdjust(driveBase,
        // AdjustmentDirection.FORWARD));
        // driveController.povDown().whileTrue(new MinorAdjust(driveBase,
        // AdjustmentDirection.BACKWARDS));
        // driveController.povUpLeft().whileTrue(new MinorAdjust(driveBase,
        // AdjustmentDirection.FRONT_LEFT));
        // driveController.povUpRight().whileTrue(new MinorAdjust(driveBase,
        // AdjustmentDirection.FRONT_RIGHT));
        // driveController.povDownLeft().whileTrue(new MinorAdjust(driveBase,
        // AdjustmentDirection.BACK_LEFT));
        // driveController.povDownRight().whileTrue(new MinorAdjust(driveBase,
        // AdjustmentDirection.BACK_RIGHT));
    }

    // #endregion
    //// #region Choosers
    public static void configureChoosers() {
        List<String> namesOfAutos = AutoBuilder.getAllAutoNames();

        autoChooser = new LoggedDashboardChooser<>("chooser");

        for (String autoName : namesOfAutos) {
            System.out.println("Loading auto: " + autoName);
            try {
                PathPlannerAuto auto = new PathPlannerAuto(autoName);
                PathPlannerAuto mirrored = new PathPlannerAuto(autoName, true);
                autoChooser.addOption(auto.getName(), auto);
                mirroredAutoMap.put(autoName, mirrored);
            } catch (Exception e) {
                DriverStation.reportError("Skipping broken auto '" + autoName + "': " + e.getMessage(),
                        e.getStackTrace()); //
            }
        }

        autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
        SmartDashboard.putData("chooser", autoChooser.getSendableChooser());

        new Trigger(RobotState::isEnabled).and(RobotState::isTeleop)
                .onTrue(new InstantCommand(() -> Robot.clearObjectPoseField("AutoPath")).ignoringDisable(true));
        new Trigger(RobotState::isDisabled).and(checkForPathChoiceUpdate).onTrue(
                new InstantCommand(() -> updateFieldFromAuto(autoChooser.get().getName())).ignoringDisable(true));

        sideChooser = new LoggedDashboardChooser<>("sideChooser");
        sideChooser.addDefaultOption("right", "right");
        sideChooser.addOption("left", "left");
        SmartDashboard.putData("sideChooser", sideChooser.getSendableChooser());

        preWarm = new LoggedDashboardChooser<>("preWarm?");
        preWarm.addDefaultOption("OFF", "OFF");
        preWarm.addOption("ON", "ON");
        SmartDashboard.putData("preWarm?", preWarm.getSendableChooser());
    }

    private static void updateFieldFromAuto(String autoName) {
        List<Pose2d> poses = new ArrayList<>();

        try {
            PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(path -> {
                poses.addAll(path.getPathPoses());
            });
        } catch (IOException | ParseException e) {
            DriverStation.reportError("Error loading auto path", e.getStackTrace()); //
        }

        Robot.setTrajectoryField("AutoPath", poses);
    }

    private static final BooleanSupplier checkForPathChoiceUpdate = new BooleanSupplier() {
        private String lastAutoName = "InstantCommand";

        @Override
        public boolean getAsBoolean() {
            if (autoChooser.get() == null)
                return false;

            String currentAutoName = autoChooser.get().getName();

            try {
                return !Objects.equals(lastAutoName, currentAutoName);
            } finally {
                lastAutoName = currentAutoName;
            }

        }
    };

    /**
     * Runs in Test mode. Lets you drive every mechanism from AdvantageScope
     * (NetworkTables "/Tuning/..."):
     * duty cycle for each motor, position (rotations) for the hood and rack and
     * pinion, RPM for the flywheel.
     * A value is applied when you change it. Everything starts stopped and stops
     * again when Test mode ends.
     */
    public static Command getTuningCommand() {
        return Commands.parallel(
                hood.tuningCommand(),
                flywheel.tuningCommand(),
                kicker.tuningCommand(),
                transfer.tuningCommand(),
                rackAndPinion.tuningCommand(),
                rollers.tuningCommand()).withName("Tuning Mode");
    }

    public static Command getAuto() {
        Command selected = autoChooser.get();
        if (sideChooser.get().equals("right"))
            return selected;
        return mirroredAutoMap.getOrDefault(selected.getName(), selected);
    }

    // #endregion
    public static void configureTrenchPaths() {
        try {
            upLeftTrenchPath = PathPlannerPath.fromPathFile("UpLeft");
            upRightTrenchPath = PathPlannerPath.fromPathFile("UpRight");
        } catch (IOException | ParseException | FileVersionException e) {
            DriverStation.reportError("Error loading trench path", e.getStackTrace());
            upLeftTrenchPath = PathPlannerPath.fromPathPoints(new ArrayList<>(),
                    PathConstraints.unlimitedConstraints(12.0), new GoalEndState(0, Rotation2d.kZero));
            upRightTrenchPath = PathPlannerPath.fromPathPoints(new ArrayList<>(),
                    PathConstraints.unlimitedConstraints(12.0), new GoalEndState(0, Rotation2d.kZero));
        }
    }

    public static Command passTrench() {
        Pose2d pose = driveBase.getPose();
        PathPlannerPath targetPath;

        if (Robot.isRedAlliance)
            pose = FlippingUtil.flipFieldPose(driveBase.getPose());
        Logger.recordOutput("Trench/Flipped Pose", pose);

        PathPlannerPath path = pose.getX() <= 4.5 ? upLeftTrenchPath : upRightTrenchPath;
        targetPath = pose.getY() >= 4 ? path : path.mirrorPath();
        Logger.recordOutput("Trench/Trench Path", targetPath.getPathPoses().toArray(new Pose2d[0]));

        PathPlannerPath flippedTargetPath = Robot.isRedAlliance ? targetPath.flipPath() : targetPath;
        Logger.recordOutput("Trench/Flipped Trench Path", flippedTargetPath.getPathPoses().toArray(new Pose2d[0]));

        return driveBase.pathFindToPathAndFollow(targetPath);
    }

    // #region Shooting Helpers
    private static final Debouncer readyToShootDebouncer = new Debouncer(0.1, DebounceType.kRising);

    /**
     * //fix: CHANGE: two changes.
     * 1. The angle check now checks if the ROBOT is aimed (the difference between
     * where it should face and where it faces).
     * REASON: it used to check |angleToHub()| < 2. angleToHub() is the direction
     * from the robot to the hub ON THE
     * FIELD, in radians, not how far the robot is from being aimed, and 2 was meant
     * as degrees. So on blue it was
     * almost always true (even when not aimed), and on red from the alliance zone
     * (direction ~180 deg = 3.14 rad)
     * it was almost never true.
     * It now compares against getShootingAngle(), the same angle AimDriving turns
     * the robot to.
     * 2. In teleop it also needs vision.isPoseConfident() (Orbit 1690 idea).
     * REASON: the hood, RPM and aim all come from the pose. If the pose is not
     * confident we would shoot blind.
     * In auto this check is skipped: the pose starts from the known auto start
     * pose, and shootWhenReady
     * feeds after 1.5 s anyway. If the pose is not confident in teleop, use the
     * manual shot (cross).
     * Every part is logged under "Shooter/" so you can see which one is blocking.
     */
    public static boolean readyToShoot() {
        // fix: PREVIOUS VERSION (before the fix):
        // return readyToShootDebouncer.calculate(Math.abs(angleToHub()) <
        // Constants.SHOOTING_ANGLE_TOLERANCE && hood.atAngle() && flywheel.atSpeed());
        double aimErrorDeg = Math
                .toDegrees(MathUtil.angleModulus(getShootingAngle() - driveBase.getHeading().getRadians()));
        boolean aimed = Math.abs(aimErrorDeg) < Constants.SHOOTING_ANGLE_TOLERANCE;
        boolean poseOk = DriverStation.isAutonomous() || vision.isPoseConfident();
        boolean hoodReady = hood.atAngle();
        boolean flywheelReady = flywheel.atSpeed();

        Logger.recordOutput("Shooter/Aim Error deg", aimErrorDeg);
        Logger.recordOutput("Shooter/Ready/Aimed", aimed);
        Logger.recordOutput("Shooter/Ready/Pose OK", poseOk);
        Logger.recordOutput("Shooter/Ready/Hood", hoodReady);
        Logger.recordOutput("Shooter/Ready/Flywheel", flywheelReady);

        return readyToShootDebouncer.calculate(aimed && poseOk && hoodReady && flywheelReady);
    }

    // fix: CHANGE: new. Ready check for the manual shot: only the hood and flywheel
    // (no pose, no angle).
    private static final Debouncer manualReadyToShootDebouncer = new Debouncer(0.1, DebounceType.kRising);

    public static boolean manualReadyToShoot() {
        return manualReadyToShootDebouncer.calculate(hood.atAngle() && flywheel.atSpeed());
    }

    private static Pose2d hubPose() {
        Pose2d hub = Constants.FieldConstants.HUB_POSITION;

        if (Robot.isRedAlliance) {
            hub = FlippingUtil.flipFieldPose(hub);
        }
        return hub;
    }

    public static double distanceFromHub() {
        return driveBase.getPose().getTranslation().getDistance(hubPose().getTranslation());
    }

    public static double angleToHub() {
        return driveBase.getAngleToPose(hubPose());
    }

    public static double passDistance() {
        return (distanceFromHub() + getDistanceToNearestTrench()) / 2;
    }

    public static double getShootingAngle() {
        if (inAllianceZone()) {
            return angleToHub();
        } else {
            return (getAngleToNearestTrench() + angleToHub()) / 2;
        }
    }

    public static boolean inAllianceZone() {
        if (Robot.isRedAlliance)
            return driveBase.getPose().getX() > Constants.FieldConstants.RED_ALLIANCE_ZONE_X;
        else
            return driveBase.getPose().getX() < Constants.FieldConstants.BLUE_ALLIANCE_ZONE_X;
    }
    // #endregion

    // #region Trench Helpers
    public static Pose2d NearestTrench() {
        Pose2d upTrench;
        Pose2d downTrench;

        if (Robot.isRedAlliance) {
            upTrench = Constants.TrenchLocations.upLeftTrench;
            downTrench = Constants.TrenchLocations.downLeftTrench;
        } else {
            upTrench = Constants.TrenchLocations.upRightTrench;
            downTrench = Constants.TrenchLocations.downRightTrench;
        }
        Translation2d robot = driveBase.getPose().getTranslation();
        double distanceToUp = robot.getDistance(upTrench.getTranslation());
        double distanceToDown = robot.getDistance(downTrench.getTranslation());
        if (distanceToUp < distanceToDown) {
            return upTrench;
        } else {
            return downTrench;
        }
    }

    public static double getDistanceToNearestTrench() {
        return driveBase.getPose().getTranslation().getDistance(NearestTrench().getTranslation());
    }

    public static double getAngleToNearestTrench() {
        return driveBase.getAngleToPose(NearestTrench());

    }

    private static final Debouncer currentJumpsInFlywheel = new Debouncer(0.6, DebounceType.kRising);

    public static Boolean areBallsGoingIntoShooter() {
        return currentJumpsInFlywheel.calculate(flywheel.getCurrent() > flyWheelConstants.CURRENT_WHEN_SHOOTING);
    }

    // #endregion

    // #region Periodic Updates
    public static void pollAlerts() {
        controllerDisconnected.set(!driveController.isConnected());
        brownoutBattery.set(RobotController.isBrownedOut());
    }

    public static boolean getPreWarm() {
        return preWarm != null && "ON".equals(preWarm.get());
    }

    public static void updateElastic() {
        Alliance alliance = Robot.isRedAlliance ? Alliance.Red : Alliance.Blue;
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Robot Velocity", driveBase.getVelocity());
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
        SmartDashboard.putNumber("PDH Voltage", ConduitApi.getInstance().getPDPVoltage());

        SmartDashboard.putBoolean("Is Hub Active", HubTracker.isActive(alliance));
        SmartDashboard.putNumber("Time left in Shift",
                Math.floor(HubTracker.timeRemainingInCurrentShift().orElseGet(() -> Second.zero()).in(Second) * 100)
                        / 100);
        SmartDashboard.putString("Current Shift",
                HubTracker.getCurrentShift().orElseGet(() -> HubTracker.Shift.AUTO).toString());

        SmartDashboard.putNumber("Fuel Approximation", fuelApproximation);
        SmartDashboard.putNumber("Distance to Hub", distanceFromHub());
        SmartDashboard.putBoolean("In Alliance Zone", inAllianceZone());
        SmartDashboard.putBoolean("Ready To Shoot", readyToShoot());
        // fix: CHANGE: new. Green = the normal shot (R1) can be used. Red = use the
        // manual shot (cross).
        SmartDashboard.putBoolean("Pose Confident", vision.isPoseConfident());
        SmartDashboard.putString("intakeState", rackAndPinion.rackAndPinionState().name());
    }
    // #endregion
}