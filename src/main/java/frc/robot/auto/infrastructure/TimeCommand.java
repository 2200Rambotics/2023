package frc.robot.auto.infrastructure;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;

public class TimeCommand implements Command {

    Timer timer;
    double duration;
    boolean isStarted;

    public TimeCommand(double duration) {
        timer = new Timer();
        this.duration = duration;
        isStarted = false;
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        if (isStarted == false) {
            timer.start();
            isStarted = true;
        }
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return timer.get() >= duration;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        timer = new Timer();
        isStarted = false;
    }

}
