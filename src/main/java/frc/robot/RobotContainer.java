package frc.robot;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.subsystems.DriveTrain.DriveBase;

public class RobotContainer {
    public static CommandPS5Controller driveController;
    public static DriveBase driveBase;

    public RobotContainer(){
        driveBase = new DriveBase();
        driveController = new CommandPS5Controller(0);

        configureDriveBindings();
    }

    public static void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(
            new StartEndCommand(() ->driveBase.setDefaultCommand(new DriveCommand(driveBase, driveController)),
            driveBase::removeDefaultCommand)
            .ignoringDisable(true));

        driveController.PS().onTrue(driveBase.resetOnlyDirection());
    }
}