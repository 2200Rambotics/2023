package frc.robot.auto.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Arm;
import frc.robot.auto.infrastructure.Command;

public class MoveArm implements Command{

    Arm arm;
    double pivotPosition;
    double extendPosition;
    int numLoops = 0;

    public MoveArm(Arm arm, double pivotPosition, double extendPosition) {
        this.arm = arm;
        this.pivotPosition = pivotPosition;
        this.extendPosition = extendPosition;
    }

    @Override
    public void run() {
        arm.setPivot(pivotPosition, 0);
        arm.setExtension(extendPosition, 0);
        numLoops++;
    }

    @Override
    public boolean isDone() {
        // TODO add moe's tavern
        // SmartDashboard.putBoolean("numloops>3", (numLoops > 100));
        // SmartDashboard.putBoolean("checkPosition", arm.checkPosition(Arm.PIVOT_MOE, Arm.EXTEND_MOE));
        return (numLoops > 3) && arm.checkPosition(Arm.PIVOT_MOE, Arm.EXTEND_MOE);
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        numLoops = 0;
    }
    
}
