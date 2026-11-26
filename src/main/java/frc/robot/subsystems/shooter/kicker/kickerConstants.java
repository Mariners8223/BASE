package frc.robot.subsystems.Shooter.kicker;

import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

public class kickerConstants {
    public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.RIO;
    public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
    public static final double GEAR_RATIO = 0.4;
    public static final int LEAD_MOTOR_CURRENT_LIMIT = 100;
    public static final int LEAD_MOTOR_CURRENT_THRESHOLD = 80;
    public static final int LEAD_MOTOR_ID = 21;
    public static final int FOLLOW_MOTOR_ID = 22;
    public static final boolean LEAD_MOTOR_IS_INVERTED = true;
    public static final boolean FOLLOW_MOTOR_INVERTED_FROM_LEAD = true;
    public static final double KICKER_EJECT_SPEED = -0.8;
    public static final double DUTY_CYCLE = 0.5;
}