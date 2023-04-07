package frc.robot.auto.commands;

import frc.robot.DriveTrain;
import frc.robot.auto.infrastructure.Command;

public class SetBalancing implements Command{

    DriveTrain drivetrain;
    boolean balanceMode = false;
    boolean ranOnce = false;

    public SetBalancing(DriveTrain drivetrain, boolean balanceMode) {
        this.drivetrain = drivetrain;
        this.balanceMode = balanceMode;

    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        drivetrain.setBalancing(balanceMode);
        ranOnce = true;
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return ranOnce;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        ranOnce = false;
    }
    
}
