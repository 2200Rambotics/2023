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
import frc.robot.auto.commands.DoBalancing;
import frc.robot.auto.commands.DriveStraight;
import frc.robot.auto.commands.FollowTrajectory;
import frc.robot.auto.commands.Intake;
import frc.robot.auto.commands.IntakeTime;
import frc.robot.auto.commands.MoveArm;
import frc.robot.auto.commands.Score;
import frc.robot.auto.commands.SetBalancing;
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

public class TrajectoryAuto implements Command {

    Sequential sequence;
    boolean firstTrajectory = true;

    public TrajectoryAuto(DriveTrain drivetrain, Arm arm, Claw claw, NavX navi, String trajectoryName,
            double maxVelocity, double maxAccel) {

        List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup(trajectoryName,
                // new PathConstraints(1.5, 2.3)); //4.0 2.3
                new PathConstraints(maxVelocity, maxAccel)); // 4.0 2.3

        List<Command> commands = new ArrayList<>(Arrays.asList(new Command[] {
                new ParallelAll(new Command[] {
                        new SwitchDriveMode(drivetrain, true),
                        new BrakeMode(drivetrain, true)
                })
        }));

        PathPlannerTrajectory lastTrajectory = null;
        for (var traj : trajectories) {
            lastTrajectory = traj;
            StopEvent startStopEvent = traj.getStartStopEvent();
            AddEventHandlersForStopEvent(drivetrain, arm, claw, commands, startStopEvent);
            if (firstTrajectory) {
                commands.add(new SetPose(traj.sample(0).poseMeters, drivetrain));
            }
            commands.add(new StepName("Drive trajectory"));
            commands.add(new FollowTrajectory(drivetrain, traj, getEventCommandMap(arm, claw, drivetrain)));
            firstTrajectory = false;
        }
        if (lastTrajectory != null) {
            AddEventHandlersForStopEvent(drivetrain, arm, claw, commands, lastTrajectory.getEndStopEvent());
        }
        sequence = new Sequential(commands.toArray(new Command[0]));

    }

    private void AddEventHandlersForStopEvent(DriveTrain drivetrain, Arm arm, Claw claw, List<Command> commands,
            StopEvent startStopEvent) {
        double waitTime = startStopEvent.waitTime;
        var events = startStopEvent.names;
        ArrayList<Command> startStopCommands = new ArrayList<>();
        startStopCommands.add(new StopDrive(drivetrain));
        for (var event : events) {
            startStopCommands.add(new EventHandler(event, getEventCommandMap(arm, claw, drivetrain)));
        }
        Command startStopCommandAggregator;
        if (startStopEvent.executionBehavior == ExecutionBehavior.PARALLEL
                || startStopEvent.executionBehavior == ExecutionBehavior.PARALLEL_DEADLINE) {
            startStopCommandAggregator = new ParallelAll(startStopCommands.toArray(new Command[0]));
        } else {
            startStopCommandAggregator = new Sequential(startStopCommands.toArray(new Command[0]));
        }
        commands.add(new WhileCommand(new TimeCommand(waitTime), startStopCommandAggregator));
    }

    private HashMap<String, Command> getEventCommandMap(Arm arm, Claw claw, DriveTrain drive) {
        HashMap<String, Command> eventCommandMap = new HashMap<String, Command>() {
            {
                put("ARMSTOW", new MoveArm(arm, Arm.PIVOT_STOW, Arm.EXTEND_STOW));
                put("ARMGROUNDPICKUP",
                        new MoveArm(arm, Arm.LVL_1_REVERSE_PIVOT_CONE_SHORT, Arm.LVL_1_REVERSE_EXTEND_CONE_SHORT));
                put("INTAKEIN", new IntakeTime(claw, .25, WheelStates.in));
                put("INTAKESLOWIN", new IntakeTime(claw, .25, WheelStates.slowIn));
                put("SCORECUBELEVEL2", new Score(Arm.LVL_2_PIVOT_CUBE, Arm.LVL_2_EXTEND_CUBE, claw, arm, false));
                put("SCORECUBELEVEL3", new Score(Arm.LVL_3_PIVOT_CUBE, Arm.LVL_3_EXTEND_CUBE, claw, arm, false));
                put("SCORECONELEVEL2", new Score(Arm.LVL_2_PIVOT_CONE_SCORE, Arm.LVL_2_EXTEND_CONE, claw, arm, false));
                put("PIVOTCUBELEVEL3", new MoveArm(arm, Arm.LVL_3_PIVOT_CUBE, Arm.EXTEND_STOW));
                put("SCORECONELEVEL3", new Score(Arm.LVL_3_PIVOT_CONE_SCORE, Arm.LVL_3_EXTEND_CONE, claw, arm, false));
                put("BALANCE", new Sequential(new Command[] {
                        new SetBalancing(drive, true),
                        new DoBalancing(drive)
                }));
            }
        };
        return eventCommandMap;
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
