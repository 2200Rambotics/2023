package frc.robot.auto.infrastructure;

public class WhileCommand implements Command {

    Command condition;
    Command command;

    public WhileCommand(Command condition, Command command) {
        this.condition = condition;
        this.command = command;
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        condition.run();
        if (condition.isDone() == false) {
            command.run();
        }

    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return condition.isDone();
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        command.reset();
        condition.reset();

    }
}
