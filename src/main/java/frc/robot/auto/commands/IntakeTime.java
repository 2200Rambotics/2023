package frc.robot.auto.commands;

import frc.robot.Claw;
import frc.robot.WheelStates;
import frc.robot.auto.infrastructure.Command;
import frc.robot.auto.infrastructure.TimeCommand;
import frc.robot.auto.infrastructure.WhileCommand;

public class IntakeTime implements Command {

    Claw claw;
    Intake intake;
    WhileCommand phile;

    public IntakeTime(Claw claw, double duration, WheelStates wheelstate) {
        this.claw = claw;
        phile = new WhileCommand(
                new TimeCommand(duration),
                new Intake(claw, wheelstate));
    }

    @Override
    public void run() {
        phile.run();
    }

    @Override
    public boolean isDone() {
        return phile.isDone();
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        phile.reset();
    }

}
