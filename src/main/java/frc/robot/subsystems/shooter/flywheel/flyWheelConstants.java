package frc.robot.subsystems.Shooter.flywheel;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersTalonFX;

public class flyWheelConstants {
    public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.RIO;
    public static final Translation2d ROBOT_TO_SHOOTER = new Translation2d(0.0, 0.0);
    public static final int MOTOR_ID = 19;  
    public static final int SEC_MOTOR_ID = 20;  
    public static final boolean IS_INVERTED = false;
    public static final boolean IS_FOLLOWER_INVERTED_FROM_LEAD = true; 

    public static final double GEAR_RATIO = 1; // motor rotations per flywheel rotation. Max flywheel speed at 12V is about 6000 / 9 = ~670 RPM
    public static final int CURRENT_LIMIT = 80;
    public static final int CURRENT_THRESHOLD = 100;
    // Gains are set per motor rotation and multiplied by GEAR_RATIO, because the controller
    // works in flywheel rotations per second. That keeps them right if GEAR_RATIO changes.
    public static final double MOTOR_KV = 0.1132; 
    public static final double MOTOR_KP = 0.16443; 
    public static final double KS = 0.03245; // volts to overcome friction
    public static final double KV = MOTOR_KV;
    public static final double KA = 0.0070555;
    public static final double KP = MOTOR_KP;
    public static final PIDFGains PID = new PIDFGains(KP, 0, 0,KV);
    public static final double RPM_TOLERANCE = 100;
    public static final double CURRENT_WHEN_SHOOTING = 115; // TODO: find the number of current while shooting
    public static double getCurrent(MarinersTalonFX motor) {
        return Math.abs(motor.getMotor().getStatorCurrent().getValueAsDouble());
    }
    private static final double[][] SHOT_TABLE = {//DISTANCE, HOOD ANGLE, RPM
        {1.8, 0.167, 1000},
        {2.3, 0.271, 1000},
        {2.8, 0.375, 1000},
        {3.3, 0.479, 1000},
        {3.8, 0.583, 1000},
        {4.3, 0.688, 1000},
        {4.8, 0.792, 1000},
        {5.3, 0.896, 1000},
        {5.8, 1.0,  1000},
    };
    public static final double MIN_TABLE_DISTANCE = 1.8;
    public static final double MAX_TABLE_DISTANCE = 5.8;
    

    private static final InterpolatingDoubleTreeMap DISTANCE_TO_HOOD_PERCENT = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap DISTANCE_TO_RPM = new InterpolatingDoubleTreeMap();
    static {
        for (int i = 0; i < SHOT_TABLE.length; i++) {
            DISTANCE_TO_HOOD_PERCENT.put(SHOT_TABLE[i][0], SHOT_TABLE[i][1]);
            DISTANCE_TO_RPM.put(SHOT_TABLE[i][0], SHOT_TABLE[i][2]);
        }
    }
    public static final double shouldMAX = 532523;
    public static final double shouldMIN = 532351;
    public static double getHoodPercent(double distanceToHub) {
        if (distanceToHub == shouldMAX) {
            return 1;
        }
        if (distanceToHub == shouldMIN) {
            return 0;
        }
        return DISTANCE_TO_HOOD_PERCENT.get(distanceToHub);
    }

    public static double getRPM(double distanceToHub) {
        return DISTANCE_TO_RPM.get(distanceToHub);
    }

    public static final Translation2d BLUE_HUB_CENTER = new Translation2d(Units.inchesToMeters(182.107), Units.inchesToMeters(158.844));
    public static final Translation2d RED_HUB_CENTER = new Translation2d(Units.inchesToMeters(469.115), Units.inchesToMeters(158.844));

    public static Translation2d getHubCenter() {
        return Robot.isRedAlliance ? RED_HUB_CENTER : BLUE_HUB_CENTER;
    }
}