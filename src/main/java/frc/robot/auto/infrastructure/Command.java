package frc.robot.auto.infrastructure;

public interface Command {
    public void run();

    public boolean isDone();

    public void reset();
}
