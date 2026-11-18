package frc.util.FastGyros;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.util.sendable.SendableBuilder;

import java.util.LinkedList;

public class PigeonIO extends GyroIO{

    private final Pigeon2 pigeon;

    /** //fix: CHANGE: new. Converts the Pigeon's g units to m/s^2 (see updateInputs). */
    private static final double G_TO_METERS_PER_SECOND_SQUARED = 9.80665;

    private final BaseStatusSignal[] signals;

    private final StatusSignal<Angle> yaw;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;
    private final StatusSignal<AngularVelocity> yawRate;


    public PigeonIO(int canID) {
        pigeon = new Pigeon2(canID);

        LinkedList<BaseStatusSignal> signals = new LinkedList<>();

        yaw = pigeon.getYaw();
        pitch = pigeon.getPitch();
        roll = pigeon.getRoll();
        accelerationX = pigeon.getAccelerationX();
        accelerationY = pigeon.getAccelerationY();
        yawRate = pigeon.getAngularVelocityZWorld();

        //fix: CHANGE: yaw and yawRate 50 Hz -> 100 Hz.
        // REASON: with a 50 Hz signal and a 50 Hz robot loop, the heading could be up to 20 ms old when read. At 100 Hz
        // it is at most 10 ms old (the max CTRE recommends on the roboRIO CAN bus). This makes odometry and the
        // heading-aided vision solve more accurate while turning. Pitch/roll/accel stay at 50 Hz (only used for filtering).
        //fix: PREVIOUS VERSION (before the fix):
        //    yaw.setUpdateFrequency(50);
        yaw.setUpdateFrequency(100);
        pitch.setUpdateFrequency(50);
        roll.setUpdateFrequency(50);
        accelerationX.setUpdateFrequency(50);
        accelerationY.setUpdateFrequency(50);
        //fix: PREVIOUS VERSION (before the fix):
        //    yawRate.setUpdateFrequency(50);
        yawRate.setUpdateFrequency(100);

        pigeon.optimizeBusUtilization();

        signals.add(yaw);
        signals.add(pitch);
        signals.add(roll);
        signals.add(accelerationX);
        signals.add(accelerationY);
        signals.add(yawRate);

        this.signals = new StatusSignal[signals.size()];

        signals.toArray(this.signals);
    }

    @Override
    public void updateInputs() {
        BaseStatusSignal.refreshAll(signals);

        inputs.angle = Rotation2d.fromDegrees(this.yaw.getValueAsDouble());
        inputs.yaw = -this.yaw.getValueAsDouble();
        inputs.pitch = this.pitch.getValueAsDouble();
        inputs.roll = this.roll.getValueAsDouble();
        inputs.yawRate = -this.yawRate.getValueAsDouble();
        //fix: CHANGE: multiply by 9.80665.
        // REASON: the Pigeon 2 reports acceleration in g, but GyroIO (and SimGyroIO) promise m/s^2,
        // so on the real robot the value was 9.8x too small.
        //fix: PREVIOUS VERSION (before the fix):
        //    inputs.accelerationX = this.accelerationX.getValueAsDouble();
        //    inputs.accelerationY = this.accelerationY.getValueAsDouble();
        inputs.accelerationX = this.accelerationX.getValueAsDouble() * G_TO_METERS_PER_SECOND_SQUARED;
        inputs.accelerationY = this.accelerationY.getValueAsDouble() * G_TO_METERS_PER_SECOND_SQUARED;
    }

    @Override
    public Rotation2d getRotation2d() {
        return inputs.angle;
    }

    @Override
    public double getYaw() {
        return inputs.yaw;
    }

    @Override
    public double getYawRate() {
        return inputs.yawRate;
    }

    @Override
    public double getPitch() {
        return inputs.pitch;
    }

    @Override
    public double getRoll() {
        return inputs.roll;
    }

    @Override
    public double getAccelerationX() {
        return inputs.accelerationX;
    }

    @Override
    public double getAccelerationY() {
        return inputs.accelerationY;
    }

    @Override
    public void reset(Rotation2d newAngle) {
        pigeon.setYaw(newAngle.getDegrees());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        pigeon.initSendable(builder);
    }
}
