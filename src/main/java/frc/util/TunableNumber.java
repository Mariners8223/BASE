package frc.util;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * A number you can type in from AdvantageScope (or any NetworkTables dashboard) while the robot runs.
 * It shows up in NetworkTables under "/Tuning/<key>".
 * In AdvantageScope: connect to the robot, turn on Tuning Mode (the slider icon in the sidebar),
 * open the "Tuning" table and edit the number.
 * The value is also logged (under NetworkInputs/Tuning/...) so it works with log replay.
 */
public class TunableNumber {
    private static final String ROOT = "/Tuning/";

    private final LoggedNetworkNumber number;
    private double lastSeenValue;

    public TunableNumber(String key, double defaultValue) {
        number = new LoggedNetworkNumber(ROOT + key, defaultValue);
        lastSeenValue = defaultValue;
    }

    public double get() {
        return number.get();
    }

    /** True once each time someone edits the value. Call it once per loop. */
    public boolean hasChanged() {
        double value = number.get();
        if (value != lastSeenValue) {
            lastSeenValue = value;
            return true;
        }
        return false;
    }

    /** Forgets old edits, so only edits made after this call count as changes. */
    public void markSeen() {
        lastSeenValue = number.get();
    }
}
