package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterInputs{


    }
    //not good
    void setDutyCycle(double desired_dutyCycle); //done(?)
    void setVelocity(double desired_Velocity); //done
    void setVoltage(double desired_Voltage);
    //void setAngle(double desired_Angle); //done

    double getVelocity();//done
    double getAcceleration();//done
    //double getAngle();

    //void shooterVision();

    void update(ShooterInputs inputs);//done

}
