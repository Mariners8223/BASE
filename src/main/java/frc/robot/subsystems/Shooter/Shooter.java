package frc.robot.subsystems.Shooter;

import frc.robot.Robot;

public class Shooter extends SubsystemBase{
    private final ShooterIO io;
    public Shooter(){
        io = 
        //Robot.isReal() ? 
        new ShooterIOReal();
        
    }

    

}
