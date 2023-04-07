package frc.robot.auto.autos;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Arm;
import frc.robot.Claw;
import frc.robot.DriveTrain;
import frc.robot.NavX;
import frc.robot.WheelStates;
import frc.robot.auto.commands.BrakeMode;
import frc.robot.auto.commands.DriveStraight;
import frc.robot.auto.commands.FollowTrajectory;
import frc.robot.auto.commands.Intake;
import frc.robot.auto.commands.IntakeTime;
import frc.robot.auto.commands.MoveArm;
import frc.robot.auto.commands.Score;
import frc.robot.auto.commands.SetPose;
import frc.robot.auto.commands.StopDrive;
import frc.robot.auto.commands.SwitchDriveMode;
import frc.robot.auto.commands.ZeroEncoders;
import frc.robot.auto.infrastructure.Command;
import frc.robot.auto.infrastructure.EventHandler;
import frc.robot.auto.infrastructure.ParallelAll;
import frc.robot.auto.infrastructure.Sequential;
import frc.robot.auto.infrastructure.StepName;
import frc.robot.auto.infrastructure.TimeCommand;
import frc.robot.auto.infrastructure.WhileCommand;

public class TestAuto implements Command {

    Sequential sequence;
    boolean firstTrajectory = true;

    public TestAuto(DriveTrain drivetrain, Arm arm, Claw claw, NavX navi) {

        sequence = new Sequential(new Command[] {
            new ParallelAll(new Command[] {
                    new SwitchDriveMode(drivetrain, true),
                    new BrakeMode(drivetrain, true)
            })
    });

    }


    @Override
    public void run() {
        sequence.run();
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        sequence.reset();
    }

}
