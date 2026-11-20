package frc.robot.subsystems.Shooter;

import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersTalonFX;

public class ShooterConstants {
    public static final double desired_dutyCycle = 6969696; //put correct number here
    public static final double desired_Angle =  2342; //insert real number
    public static final double desired_Velocity = 123123;
    public static final double desired_Voltage8 = 123;

    public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
    public static final int LEAD_MOTOR_ID = 123;
    public static final int FOLLOW_MOTOR_ID = 1234;

    public static final boolean IS_LEAD_INVERT = false;
    public static final boolean IS_FOLLOW_INVERT = true;

    public static final double GEAR_RATIO = 9;
    public static final double GEAR_REDUCTION = 1 / GEAR_RATIO;

    

}
