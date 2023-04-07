package frc.robot.auto.commands;

import frc.robot.DriveTrain;
import frc.robot.NavX;
import frc.robot.auto.infrastructure.Command;



public class Turn implements Command{

    DriveTrain drivetrain;
    NavX navi;
    double angle;
    double marginOfError;

    public Turn(DriveTrain drivetrain, NavX navi, double angle, double marginOfError) {
        this.drivetrain = drivetrain;
        this.navi = navi;
        this.angle = angle;
        this.marginOfError = marginOfError;
    }

    @Override
    public void run() {
        drivetrain.turnToZero(angle);
        drivetrain.drive(0, 0, 0, 0);

    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return (Math.abs(angle - navi.currentHeading()) < marginOfError) && Math.abs(navi.ahrs.getRate()) < 10;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }
    
}
