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

    public static final double GEAR_RATIO = 225; // TODO: check. With ratio 3 and KV 0.36, 12V tops out near 2000 RPM, but SHOT_TABLE asks for 2500-3870
    public static final int CURRENT_LIMIT = 80;
    public static final int CURRENT_THRESHOLD = 100;
    public static final double KS = 0.1; 
    public static final double KV = 0.36;  
    public static final PIDFGains PID = new PIDFGains(0.05, 0, 0, KV);
    public static final double RPM_TOLERANCE = 100;
    public static final double CURRENT_WHEN_SHOOTING = 115; // TODO: find the number of current while shooting
    public static double getCurrent(MarinersTalonFX motor) {
        return Math.abs(motor.getMotor().getStatorCurrent().getValueAsDouble());
    }
    private static final double[][] SHOT_TABLE = {
        {1.8, 0.167, 200},
        {2.3, 0.271, 200},
        {2.8, 0.375, 200},
        {3.3, 0.479, 200},
        {3.8, 0.583, 200},
        {4.3, 0.688, 200},
        {4.8, 0.792, 200},
        {5.3, 0.896, 200},
        {5.8, 1.0,   200},
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

    public static double getHoodPercent(double distanceToHub) {
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