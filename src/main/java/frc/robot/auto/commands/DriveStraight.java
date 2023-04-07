package frc.robot.auto.commands;

import frc.robot.DriveTrain;
import frc.robot.auto.infrastructure.Command;

public class DriveStraight implements Command{

    DriveTrain drivetrain;
    double startDistance;
    double targetDistance;
    double targetCompass;
    double speed;
    double slowDown;
    double driveMoE;
    double headingMoE;
    double maxTurnSpeed;

    boolean completed = false;
    boolean ran = false;

    public DriveStraight(DriveTrain drivetrain, double targetDistance, 
            double targetCompass, double speed, double slowDown, double driveMoE, double headingMoE, double maxTurnSpeed) {
        
        this.drivetrain = drivetrain;
        this.targetDistance = targetDistance;
        this.targetCompass = targetCompass;
        this.speed = speed;
        this.slowDown = slowDown;
        this.driveMoE= driveMoE;
        this.headingMoE = headingMoE;
        this.maxTurnSpeed = maxTurnSpeed;
    }


    @Override
    public void run() {
        // Gets the start distance the first time run is called
        if (!ran) { // Runs if this is the first time run has run
            startDistance = drivetrain.getAverageDistance();
            ran = true;
        }

        completed = drivetrain.driveDistance(startDistance, targetDistance, targetCompass, speed, slowDown, driveMoE, headingMoE, maxTurnSpeed);
    }

    @Override
    public boolean isDone() {
        return completed;
    }


    @Override
    public void reset() {
        // TODO Auto-generated method stub
        ran = false;
        completed = false;
    }
    


}
