package frc.robot.auto.commands;

import frc.robot.DriveTrain;
import frc.robot.auto.infrastructure.Command;

public class BrakeMode implements Command{

    DriveTrain drivetrain;
    int counts = 0;
    boolean brake;

    public BrakeMode(DriveTrain drivetrain, boolean brake) {
        this.drivetrain = drivetrain;
        this.brake = brake;
    }


    @Override
    public void run() {
        // TODO Auto-generated method stub
        if (brake) {
            drivetrain.motorBrakeMode();
        } else {
            drivetrain.motorCoastMode();
        }
        
        counts++;
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return counts > 2;
    }


    @Override
    public void reset() {
        // TODO Auto-generated method stub
        counts = 0;
    }
    
}
