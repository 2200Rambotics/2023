package frc.robot.auto.commands;

import frc.robot.DriveTrain;
import frc.robot.Limelight;
import frc.robot.auto.infrastructure.Command;

public class LimelightDrive implements Command{

    DriveTrain drivetrain;
    Limelight limelight;
    double speed;
    double heading;
    double distance;
    double startDistance;
    boolean firstTime = true;

    public LimelightDrive(DriveTrain drivetrain, Limelight limelight, double speed, double distance, double heading) {
        this.limelight = limelight;
        this.speed = speed;
        this.heading = heading;
        this.drivetrain = drivetrain;
        this.distance = distance;
    }

    @Override
    public void run() {
        if (firstTime) {
            startDistance = drivetrain.getAverageDistance();
            firstTime = false;
        }

        double x = -DriveTrain.clamp(limelight.tx * 0.03, -1, 1);

        drivetrain.targetHeading = heading;
        
        drivetrain.mechanumPID(x, speed, 0);
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return Math.abs((startDistance + distance) - drivetrain.getAverageDistance()) < 0.5; // Margin of error on right side
        // Abs (real target - current) < MoE
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        firstTime = true;
    }
    
}
