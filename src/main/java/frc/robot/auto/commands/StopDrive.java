package frc.robot.auto.commands;
import frc.robot.DriveTrain;
import frc.robot.auto.infrastructure.Command;

public class StopDrive implements Command {

    DriveTrain drivetrain;
    boolean ranOnce = false;

    public StopDrive(DriveTrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    @Override
    public void run() {
        // TODO Auto-generated method stub
        drivetrain.arcadeDrive(0, 0);
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
