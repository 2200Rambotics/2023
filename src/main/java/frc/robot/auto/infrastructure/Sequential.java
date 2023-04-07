package frc.robot.auto.infrastructure;

public class Sequential implements Command {

    Command[] commands;

    int i;

    public Sequential(Command[] commands) {
        this.commands = commands;

    }

    @Override
    public void run() {
        if (i < commands.length) {
            Command command = commands[i];
            command.run();
            if (command.isDone() == true) {
                i++;
            }
        }

    }

    @Override
    public boolean isDone() {
        if (i < commands.length) {
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        i = 0;
        for (var command : commands) {
            command.reset();
        }
    }

}
