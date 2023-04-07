package frc.robot.auto.commands;

import frc.robot.DriveTrain;
import frc.robot.auto.infrastructure.Command;

public class DoBalancing implements Command{

    DriveTrain drivetrain;

    public DoBalancing(DriveTrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    @Override
    public void run() {
        drivetrain.drive(0, 0, 0, 0);
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }
    
}
