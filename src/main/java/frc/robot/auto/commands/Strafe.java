package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DriveTrain;
import frc.robot.auto.infrastructure.Command;

public class Strafe implements Command {
    double speed;
    double distance;
    double heading;
    DriveTrain drive;
    double MoE;
    double x;
    double y;
    boolean ran = false;
    double startDistance;
    double targetDistance;

    public Strafe(double x, double y, double distance, double MoE, DriveTrain drive, double heading){
        this.x = x;
        this.y = y;
        this.distance = distance;
        this.drive = drive;
        this.MoE = MoE;
        this.heading = heading;
    }

    @Override
    public void run() {
        if (!ran) { // Runs if this is the first time run has run
            startDistance = drive.getAverageStrafeDistance();
            ran = true;
        }
        targetDistance = distance + startDistance;

        drive.strafeDistance(x, -y, targetDistance, MoE, heading);
        SmartDashboard.putNumber("Target Strafe Distance", targetDistance);
        SmartDashboard.putNumber("Strafe Distance", drive.getAverageStrafeDistance());
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        if (Math.abs(targetDistance - (drive.getAverageStrafeDistance())) < MoE) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        ran = false;
    }
    
}
