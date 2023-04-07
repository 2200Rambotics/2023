package frc.robot.auto.commands;

import frc.robot.LimeState;
import frc.robot.Limelight;
import frc.robot.auto.infrastructure.Command;

/*
 * Make sure to set pipeline to OFF when not in use (BALANCING!!!)
 */
public class LimelightSetPipeline implements Command{

    Limelight limelight;
    LimeState limestate;
    boolean ranOnce = false;

    public LimelightSetPipeline (Limelight limelight, LimeState limestate) {
        this.limelight = limelight;
        this.limestate = limestate;
    }

    @Override
    public void run() {
        limelight.setPipeline(limestate);
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
