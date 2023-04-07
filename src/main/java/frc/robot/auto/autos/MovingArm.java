package frc.robot.auto.autos;

import javax.management.timer.Timer;

import org.ejml.equation.Sequence;
import frc.robot.auto.infrastructure.Sequential;
import frc.robot.Arm;
import frc.robot.auto.commands.MoveArm;
import frc.robot.auto.infrastructure.Command;


public class MovingArm implements Command{

    Sequential sequence;
    boolean fwdFlipFlop = true;
    Arm arm;
    Command command;

    public MovingArm (Arm arm) {
        this.arm = arm;
        command = new MoveArm(arm, Arm.LVL_1_REVERSE_PIVOT_CUBE_SHORT, Arm.LVL_1_REVERSE_EXTEND_CUBE_SHORT);;
    }

    @Override
    public void run() {
        if (command.isDone()) {
            if (fwdFlipFlop) {
                command = new MoveArm(arm, Arm.LVL_1_REVERSE_PIVOT_CUBE_SHORT, Arm.LVL_1_REVERSE_EXTEND_CUBE_SHORT);
                fwdFlipFlop = false;
            } else {
                command = new MoveArm(arm, Arm.LVL_1_PIVOT_CUBE, Arm.LVL_1_EXTEND_CUBE);
                fwdFlipFlop = true;
            }
        }
        command.run();
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        sequence.reset();
        fwdFlipFlop = false;
        command.reset();
    }
    
}
