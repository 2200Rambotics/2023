package frc.robot.auto.commands;

import java.util.Set;

import frc.robot.DriveTrain;
import frc.robot.auto.infrastructure.Command;
import frc.robot.auto.infrastructure.TimeCommand;

public class SwitchDriveMode implements Command{

    DriveTrain drive;
    boolean isInTractionMode;
    TimeCommand timer;


    public SwitchDriveMode(DriveTrain drive, boolean isInTractionMode) {
        this.drive = drive;
        this.isInTractionMode = isInTractionMode;
        timer = new TimeCommand(0.1);
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        drive.setDriveState(isInTractionMode);
        timer.run();
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return timer.isDone();
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        timer.reset();
    }

    

   
    
}
