package frc.robot.auto.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.DriveTrain;
import frc.robot.auto.infrastructure.Command;

public class SetPose implements Command {

    Pose2d pose;
    DriveTrain drive;
    
    public SetPose(Pose2d pose, DriveTrain drive) {
        this.pose = pose;
        this.drive = drive;
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        drive.resetOdometryPose(pose);
        
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }
    
}
