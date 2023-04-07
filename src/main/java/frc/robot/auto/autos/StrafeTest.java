package frc.robot.auto.autos;

import frc.robot.Arm;
import frc.robot.Claw;
import frc.robot.DriveTrain;
import frc.robot.NavX;
import frc.robot.auto.commands.DoBalancing;
import frc.robot.auto.commands.DriveStraight;
import frc.robot.auto.commands.Score;
import frc.robot.auto.commands.SetBalancing;
import frc.robot.auto.commands.StopDrive;
import frc.robot.auto.commands.Strafe;
import frc.robot.auto.commands.SwitchDriveMode;
import frc.robot.auto.infrastructure.Command;
import frc.robot.auto.infrastructure.Sequential;
import frc.robot.auto.infrastructure.StepName;

public class StrafeTest implements Command{

    
    DriveTrain drive;
    Sequential sequence;

    public StrafeTest(DriveTrain drive) {
       
        this.drive = drive;

        sequence = new Sequential(new Command[] {
            new StepName("Mechanum mode"),
            new SwitchDriveMode(drive, false),
            new StepName("Strafe 1"),
            new Strafe(.5, .2, 4, .3, drive, 0),
            new StepName("Strafe 2"),
            new Strafe(-.5, -0.2, -4, .3, drive, 0),
            new StopDrive(drive),
            new StepName("Done")
        });

    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        
        sequence.run();
        
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        sequence.reset();
    }
    
}
