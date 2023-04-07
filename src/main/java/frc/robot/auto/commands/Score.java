package frc.robot.auto.commands;

import frc.robot.Arm;
import frc.robot.Claw;
import frc.robot.WheelStates;
import frc.robot.auto.infrastructure.Command;
import frc.robot.auto.infrastructure.Sequential;
import frc.robot.auto.infrastructure.StepName;

public class Score implements Command {
    boolean isCone = false;
    boolean isReverse = false;
    int lvl = 0;
    Claw claw;
    Arm arm;
    double extendPosition;
    double pivotPosition;
    Command score;

    public Score(double pivotPosition, double extendPosition, Claw claw, Arm arm, boolean quickScore) {
        this.extendPosition = extendPosition;
        this.pivotPosition = pivotPosition;
        this.claw = claw;
        this.arm = arm;
        if (quickScore) {
            score = new Sequential(new Command[] {
                    new MoveArm(arm, pivotPosition, extendPosition),
                    new IntakeTime(claw, .2, WheelStates.out),
            });
        } else {
            score = new Sequential(new Command[] {
                    new MoveArm(arm, pivotPosition, extendPosition),
                    new IntakeTime(claw, .2, WheelStates.out),
                    new MoveArm(arm, Arm.PIVOT_STOW, Arm.EXTEND_STOW)
            });
        }
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        score.run();
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return score.isDone();
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        score.reset();
    }

}
