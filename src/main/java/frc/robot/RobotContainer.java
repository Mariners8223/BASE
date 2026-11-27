
package frc.robot;
 
import static edu.wpi.first.units.Units.Second;
 

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Vision.Vision;
import frc.util.HubTracker;
import frc.robot.subsystems.DriveTrain.DriveBaseSYSID;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.conduit.ConduitApi;
 
    public class RobotContainer {
    
    public static Alert controllerDisconnected = new Alert("Drive Controller Disconnect", AlertType.kError);
    public static Alert brownoutBattery = new Alert("BROWNOUT", AlertType.kError);
 
    public static CommandPS5Controller driveController;
    public static DriveBase driveBase;
    public static DriveBaseSYSID driveBaseSYSID;
 
    public static Runnable fuelIncrementer;
    public static int fuelApproximation = 0;
    public static Supplier<Command> cross;
    public static Supplier<Command> square;
    public static Supplier<Command> circle;


    public static double crossdouble = 0;
    public static double circledouble = 0;
    public static double squaredouble = 0;





    //#region Premade Commands


    //#endregion
 
    public RobotContainer() {
        driveController = new CommandPS5Controller(0);
        driveBase = new DriveBase();
        driveBaseSYSID = new DriveBaseSYSID(driveBase, driveController);
        configureDriveBindings();
// CROSS - Drive motors
cross = () -> {
    if (crossdouble == 0) {
        return driveBaseSYSID.getDriveMotorsRoutineQuasistatic(SysIdRoutine.Direction.kForward)
            .finallyDo(interrupted -> {
                if (!interrupted) crossdouble = 1;
            });
    }
    else if (crossdouble == 1) {
        return driveBaseSYSID.getDriveMotorsRoutineQuasistatic(SysIdRoutine.Direction.kReverse)
            .finallyDo(interrupted -> {
                if (!interrupted) crossdouble = 2;
            });
    }
    else if (crossdouble == 2) {
        return driveBaseSYSID.getDriveMotorsRoutineDynamic(SysIdRoutine.Direction.kForward)
            .finallyDo(interrupted -> {
                if (!interrupted) crossdouble = 3;
            });
    }
    else {
        return driveBaseSYSID.getDriveMotorsRoutineDynamic(SysIdRoutine.Direction.kReverse)
            .finallyDo(interrupted -> {
                if (!interrupted) crossdouble = 0;
            });
    }
};
    driveController.circle().whileTrue(new SequentialCommandGroup(
        driveBaseSYSID.getThetaRoutineDynamic(SysIdRoutine.Direction.kForward),
        driveBaseSYSID.getThetaRoutineDynamic(SysIdRoutine.Direction.kReverse),
        driveBaseSYSID.getThetaRoutineQuasistatic(SysIdRoutine.Direction.kForward),
        driveBaseSYSID.getThetaRoutineQuasistatic(SysIdRoutine.Direction.kReverse)
    ));


// SQUARE - Steer motors
square = () -> {
    if (squaredouble == 0) {
        return driveBaseSYSID.getSteerMotorsRoutineDynamic(SysIdRoutine.Direction.kForward)
            .finallyDo(interrupted -> {
                if (!interrupted) squaredouble = 1;
            });
    }
    else if (squaredouble == 1) {
        return driveBaseSYSID.getSteerMotorsRoutineDynamic(SysIdRoutine.Direction.kReverse)
            .finallyDo(interrupted -> {
                if (!interrupted) squaredouble = 2;
            });
    }
    else if (squaredouble == 2) {
        return driveBaseSYSID.getSteerMotorsRoutineQuasistatic(SysIdRoutine.Direction.kForward)
            .finallyDo(interrupted -> {
                if (!interrupted) squaredouble = 3;
            });
    }
    else {
        return driveBaseSYSID.getSteerMotorsRoutineQuasistatic(SysIdRoutine.Direction.kReverse)
            .finallyDo(interrupted -> {
                if (!interrupted) squaredouble = 0;
            });
    }
};


// CIRCLE - Theta
// circle = () -> {
//     if (circledouble == 0) {
//         return driveBaseSYSID.getThetaRoutineDynamic(SysIdRoutine.Direction.kForward)
//             .finallyDo(interrupted -> {
//                 if (!interrupted) circledouble = 1;
//             });
//     }
//     else if (circledouble == 1) {
//         return driveBaseSYSID.getThetaRoutineDynamic(SysIdRoutine.Direction.kReverse)
//             .finallyDo(interrupted -> {
//                 if (!interrupted) circledouble = 2;
//             });
//     }
//     else if (circledouble == 2) {
//         return driveBaseSYSID.getThetaRoutineQuasistatic(SysIdRoutine.Direction.kForward)
//             .finallyDo(interrupted -> {
//                 if (!interrupted) circledouble = 3;
//             });
//     }
//     else {
//         return driveBaseSYSID.getThetaRoutineQuasistatic(SysIdRoutine.Direction.kReverse)
//             .finallyDo(interrupted -> {
//                 if (!interrupted) circledouble = 0;
//             });
//     }
// };


// Button bindings
driveController.cross().whileTrue(cross.get());
driveController.square().whileTrue(square.get());
//driveController.circle().whileTrue(circle.get());
        
    }
 
    //#region Commands



    //#endregion
 
    //#region Bindings
        
    public static void configureDriveBindings() {
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(
            new StartEndCommand(() -> driveBase.setDefaultCommand(new DriveCommand(driveBase, driveController)),
            driveBase::removeDefaultCommand)
            .ignoringDisable(true));
 
        driveController.PS().onTrue(driveBase.resetOnlyDirection());
 
        driveController.povRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.RIGHT));
        driveController.povLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.LEFT));
        driveController.povUp().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FORWARD));
        driveController.povDown().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACKWARDS));
        driveController.povUpLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_LEFT));
        driveController.povUpRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_RIGHT));
        driveController.povDownLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_LEFT));
        driveController.povDownRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_RIGHT));
    }
    //#endregion
    ////#region Choosers
    public static void pollAlerts() {
        controllerDisconnected.set(!driveController.isConnected());
        brownoutBattery.set(RobotController.isBrownedOut());
    }

    public static void updateElastic() {
        Alliance alliance = Robot.isRedAlliance ? Alliance.Red : Alliance.Blue;
        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Robot Velocity", driveBase.getVelocity());
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
        SmartDashboard.putNumber("PDH Voltage", ConduitApi.getInstance().getPDPVoltage());
 
        SmartDashboard.putBoolean("Is Hub Active", HubTracker.isActive(alliance));
 
        SmartDashboard.putNumber("Fuel Approximation", fuelApproximation);


    }
    //#endregion
}