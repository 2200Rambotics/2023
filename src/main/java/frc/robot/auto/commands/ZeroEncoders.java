package frc.robot.auto.commands;

import frc.robot.DriveTrain;
import frc.robot.auto.infrastructure.Command;

public class ZeroEncoders implements Command{

    DriveTrain drivetrain;
    int counts = 0;

    public ZeroEncoders(DriveTrain drivetrain) {
        this.drivetrain = drivetrain;

    }


    @Override
    public void run() {
        // TODO Auto-generated method stub
        if(counts == 0){
            drivetrain.zeroEncoders();
        }
        counts++;
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return counts > 3;
    }


    @Override
    public void reset() {
        // TODO Auto-generated method stub
        counts = 0;
    }
    
}
