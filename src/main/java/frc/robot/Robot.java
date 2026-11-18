// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.Elastic;


public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    public static boolean isRedAlliance = false;

    private static final Field2d field = new Field2d();


    public Robot() {
        Logger.recordMetadata("ProjectName", "MyProject");

        //fix: CHANGE: three cases instead of two (real / sim / replay).
        // REASON: before, anything that wasn't the real robot went into log REPLAY and asked for a log file, so a
        // normal simulation could not run. Now replay only happens when Constants.ROBOT_TYPE == REPLAY.
        if (isReal()) {
            //fix: PREVIOUS VERSION (before the fix):
            //    Logger.addDataReceiver(new WPILOGWriter()); 
            //    Logger.addDataReceiver(new NT4Publisher()); 
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        }
        else if (frc.robot.Constants.ROBOT_TYPE != frc.robot.Constants.RobotType.REPLAY) {
            Logger.addDataReceiver(new NT4Publisher()); // plain simulation: publish to NetworkTables for AdvantageScope
        }
        else {
            setUseTiming(false); 
            String logPath = LogFileUtil.findReplayLog(); 
            Logger.setReplaySource(new WPILOGReader(logPath)); 
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        SignalLogger.enableAutoLogging(false);
        DataLogManager.stop();

        updateAlliance();

        Logger.start();
        new RobotContainer();

        PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("Pathplanner/Current Path Target", pose));
        PathPlannerLogging.setLogActivePathCallback(path -> Logger.recordOutput("Pathplanner/Current Path", path.toArray(new Pose2d[0])));

        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

        Elastic.selectTab(1);
    }


    private static void updateAlliance() {
        isRedAlliance = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static void clearObjectPoseField(String name) {
        field.getObject(name).setPoses();
    }

    public static void setTrajectoryField(String name, List<Pose2d> poses) {
        field.getObject(name).setPoses(poses);
    }

    public static void setRobotPoseOnField(Pose2d pose) {
        field.setRobotPose(pose);
    }

    @Override
    public void robotInit(){
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        RobotContainer.pollAlerts();
        RobotContainer.updateElastic();
    }

    @Override
    public void disabledPeriodic() {
        updateAlliance();
    }

    @Override
    public void autonomousInit() {
        Elastic.selectTab(1);
        updateAlliance();

        autonomousCommand = RobotContainer.getAuto();
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousExit(){
        Elastic.selectTab(0);
    }

    @Override
    public void teleopInit() {
        updateAlliance(); 
        Robot.clearObjectPoseField("AutoPath");
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

}
