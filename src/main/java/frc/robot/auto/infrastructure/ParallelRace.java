package frc.robot.auto.infrastructure;

public class ParallelRace implements Command {

    Command[] commands;

    public ParallelRace(Command[] commands) {
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
            if (command.isDone()) {
                return true;
            }

        }
        return false;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        for (var command : commands) {
            command.reset();
        }
    }

}
