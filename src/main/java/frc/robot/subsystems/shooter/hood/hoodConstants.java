package frc.robot.subsystems.Shooter.hood;

import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.PIDFGains;

public class hoodConstants {
    public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.RIO;
    //fix: CHANGE: new constant. The PID's "at setpoint" tolerance, in HOOD rotations (0.002 = about 0.7 degrees).
    public static final double PID_TOLERANCE_IN_HOOD_ROTATIONS = 0.002;
    //fix: CHANGE: was "new PIDFGains(0.05, 0, 0)". Now (kP, kI, kD, kF, tolerance, iZone).
    // REASON 1: the 3-argument constructor sets the tolerance to +infinity. Then pidController.atSetpoint()
    // is always true, and the RIO controller only sends the feed forward (0 V). The PID never moved the hood
    // or pulled it back when it went past MAX.
    // REASON 2: kP is in volts per HOOD rotation (positions are divided by GEAR_RATIO = 225). With kP = 0.05
    // and the full error of 0.08 rotations, the output was 0.004 V, which is nothing.
    // kP = 20 is only a STARTING value: tune it on the robot. Output is still capped at +-HOOD_SPEED volts.
    public static final PIDFGains PID = new PIDFGains(20, 0, 0, 0, PID_TOLERANCE_IN_HOOD_ROTATIONS, 0);
    public static final int MOTOR_ID = 18; // TODO: real CAN ID 
    public static final int CURRENT_LIMIT = 100;
    public static final int CURRENT_THRESHOLD = 80;
    public static final boolean IS_INVERTED = false;
    public static final double HOOD_SPEED = 0.75 * 12; //duty cycle 0.1 -> * 12 to volt
    // Max duty cycle you can type in from AdvantageScope tuning (same limit as HOOD_SPEED)
    public static final double TUNING_MAX_DUTY_CYCLE = HOOD_SPEED / 12;
    public static final double GEAR_RATIO = 225;

    public static final double MAX_ANGLE_MOVEMENT_IN_MOTOR_ROTATIONS = 0.37; 
    public static final double POSITION_TOLERANCE_IN_MOTOR_ROTATIONS = 0.01;

    // Homing (runs once on the first enable): drive the hood closed at -HOMING_DUTY_CYCLE until the stator current
    // jumps (hood hit the hard stop), then stop and zero the encoder.
    public static final double HOMING_DUTY_CYCLE = 0.1; // TODO: tune on the robot
    public static final double HOMING_CURRENT_THRESHOLD_AMPS = 30; 
    public static final double HOMING_CURRENT_DEBOUNCE_SECONDS = 0.1; // current must stay high this long (filters noise)
    public static final double HOMING_IGNORE_START_SECONDS = 0.5;
    public static final double HOMING_TIMEOUT_SECONDS = 10;

    public static final double SYSID_RAMP_VOLTS_PER_SECOND = 0.5;
    public static final double SYSID_STEP_VOLTS = 2;
    public static final double SYSID_TIMEOUT_SECONDS = 4;
    public static final double SYSID_LIMIT_MARGIN_IN_MOTOR_ROTATIONS = 0.5;

}