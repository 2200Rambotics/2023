package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Accelerometer, Gyroscope, & Magnetometer */
public class NavX implements Runnable {
    double startingHeading;

    public AHRS ahrs;
    public double curHeading;

    public NavX(){
        ahrs = new AHRS();
    }

    public void run(){
        while (true) {
            curHeading = ahrs.getAngle() - startingHeading; //Returns current heading of robot
            SmartDashboard.putNumber("Yaw", ahrs.getYaw());
            SmartDashboard.putNumber("Pitch", ahrs.getPitch());
            SmartDashboard.putNumber("Roll", ahrs.getRoll());
            SmartDashboard.putNumber("Compass", ahrs.getCompassHeading());
            SmartDashboard.putNumber("Angle", currentHeading());
            SmartDashboard.putNumber("Starting Heading", startingHeading);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
        
    }

    public double currentHeading() {
        return curHeading; //Returns current heading of robot
    }

    public Rotation2d currentRotation() {
        return new Rotation2d(Math.toRadians(-currentHeading()));
    }

    public double getPitch(){
        return ahrs.getPitch();
    }

    public void setStartingHeading() {
        startingHeading = ahrs.getAngle();
    }
}
