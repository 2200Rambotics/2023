package frc.robot.auto.commands;

import frc.robot.Claw;
import frc.robot.WheelStates;
import frc.robot.auto.infrastructure.Command;

public class Intake implements Command{

    Claw claw;
    double speed;
    WheelStates wheelstate;

    public Intake(Claw claw, WheelStates wheelstate) {
        this.claw = claw;
        this.wheelstate = wheelstate;
    }

    @Override
    public void run() {
        claw.wheelState = wheelstate;
    }

    @Override
    public boolean isDone() {
        return true;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        
    }
}
