package frc.robot.auto.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.DriveTrain;
import frc.robot.auto.infrastructure.Command;

public class FollowTrajectory implements Command {

    private static final int MAX_VOLTS = 12;

    private static final double MAX_SPEED = 4.0;
    DriveTrain drive;
    PathPlannerTrajectory traj;
    int runCounts = 0;
    Timer runTime;
    Timer dtt;
    double prevLeftSpeed, prevRightSpeed;
    double p = 0;
    double i = 0;
    double d = 0;
    PIDController m_leftController = new PIDController(p, i, d);
    PIDController m_rightController = new PIDController(p, i, d);
    HashMap<String, Command> eventCommandMap;

    public FollowTrajectory(DriveTrain drive, PathPlannerTrajectory traj, HashMap<String, Command> eventCommandMap) {
        this.drive = drive;
        this.traj = traj;
        runTime = new Timer();
        dtt = new Timer();
        this.eventCommandMap = eventCommandMap;
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        if (runCounts == 0) {
            runTime.start();
            dtt.start();
        }
        List<EventMarker> markers = traj.getMarkers();
        for (var marker : markers) {
            if (runTime.get() > marker.timeSeconds) {
                for (var name : marker.names) {
                    if (eventCommandMap.containsKey(name)){
                        Command command = eventCommandMap.get(name);
                        if (!command.isDone()) {
                            command.run();
                        }
                    }
                }
            }
        }
        // markers.get(0).

        Trajectory.State goal = traj.sample(runTime.get());
        double v = goal.velocityMetersPerSecond;
        double a = goal.accelerationMetersPerSecondSq;
        ChassisSpeeds cs = drive.ramsete.calculate(drive.driveOdometry.getPoseMeters(), goal);
        DifferentialDriveWheelSpeeds wheelSpeeds = drive.driveKinematics.toWheelSpeeds(cs);
        double leftVoltage = wheelSpeeds.leftMetersPerSecond / MAX_SPEED * MAX_VOLTS;
        double rightVoltage = wheelSpeeds.rightMetersPerSecond / MAX_SPEED * MAX_VOLTS;

        var leftSpeedSetpoint = wheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = wheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        double currentLeftSpeed = drive.LF1.getVelocity();
        double currentRightSpeed = drive.RF1.getVelocity();

        double dt = dtt.get();
        double calcAccelLeft = (leftSpeedSetpoint - prevLeftSpeed) / dt;
        double leftFeedforward = drive.driveFeedforward.calculate(
                leftSpeedSetpoint);
        SmartDashboard.putNumber("Trajectory real left velocity", currentLeftSpeed);
        SmartDashboard.putNumber("Trajectory desired left velocity", leftSpeedSetpoint);
        SmartDashboard.putNumber("Trajectory calc left acceleration", calcAccelLeft);
        SmartDashboard.putNumber("Trajectory calc left ff", leftFeedforward);


        double rightFeedforward = drive.driveFeedforward.calculate(
                rightSpeedSetpoint);

        leftOutput = leftFeedforward
                + m_leftController.calculate(currentLeftSpeed, leftSpeedSetpoint);

        rightOutput = rightFeedforward
                + m_rightController.calculate(
                        currentRightSpeed, rightSpeedSetpoint);




        drive.driveVoltage(leftOutput, rightOutput);

        prevLeftSpeed = leftSpeedSetpoint;
        prevRightSpeed = rightSpeedSetpoint;
        runCounts++;
        dtt.restart();
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return traj.getTotalTimeSeconds() < runTime.get();
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        runTime = new Timer();
        dtt = new Timer();
        runCounts = 0;
        prevLeftSpeed = 0;
        prevRightSpeed = 0;
    }

}
