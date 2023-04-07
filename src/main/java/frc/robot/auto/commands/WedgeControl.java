package frc.robot.auto.commands;

import frc.robot.Wedge;
import frc.robot.auto.infrastructure.Command;

public class WedgeControl implements Command{

    Wedge wedge;
    boolean requestDown;
    boolean requestFront;
    boolean ranOnce = false;

    public WedgeControl(Wedge wedge, boolean requestDown, boolean requestFront) {
        this.wedge = wedge;
        this.requestDown = requestDown;
        this.requestFront = requestFront;
    }

    @Override
    public void run() {
        if(requestDown) {
            if(requestFront) {
                wedge.wedgeAuto(requestDown, requestFront);
            } 
        }
        ranOnce = true;
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return ranOnce;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        ranOnce = false;
    }
    
}
