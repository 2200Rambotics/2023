package frc.robot.auto.infrastructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StepName implements Command {
    String name;
    boolean ran;

    public StepName(String name){
        this.name = name;
        ran = false;
    } 

    @Override
    public void run() {
        // TODO Auto-generated method stub
        SmartDashboard.putString("Current Auto Step", name);
        ran = true;
        
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return ran;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        ran = false;
    }
    
}
