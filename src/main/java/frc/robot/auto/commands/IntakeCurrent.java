package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Claw;
import frc.robot.WheelStates;
import frc.robot.auto.infrastructure.Command;

public class IntakeCurrent implements Command{

    Claw claw;
    WheelStates wheelstate;
    double currentLimit;
    boolean firstRun = true;
    Timer timer;
    double timeout;

    public IntakeCurrent(Claw claw, WheelStates wheelstate, double timeout) {
        this.claw = claw;
        this.wheelstate = wheelstate;
        timer = new Timer();
        this.timeout = timeout;
    }

    @Override
    public void run() {
        if (firstRun) {
            timer.start();
            firstRun = false; // TODO: Do this
        }
        claw.wheelState = wheelstate;
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return timer.get() > 0.2 && claw.checkWeelsCurrent() || timer.get() > timeout;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        timer = new Timer();
        firstRun = true;
    }
    
}
