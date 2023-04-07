package frc.robot.auto.infrastructure;

public class ParallelAll implements Command {

    Command[] commands;

    public ParallelAll(Command[] commands) {
        this.commands = commands;

    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        for (int i = 0; i < commands.length; i++) {
            Command command = commands[i];
            command.run();
        }

    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        for (int i = 0; i < commands.length; i++) {
            Command command = commands[i];
            if (command.isDone() == false) {
                return false;
            }

        }
        return true;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        for (var command : commands) {
            command.reset();
        }
    }

}
