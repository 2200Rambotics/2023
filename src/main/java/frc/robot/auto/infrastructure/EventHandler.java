package frc.robot.auto.infrastructure;

import java.util.HashMap;

public class EventHandler implements Command {

    String name;
    HashMap<String, Command> eventCommandMap;
    boolean isDoneCommand = false;
    boolean firstTime = true;
    Command command;
    
    public EventHandler(String name, HashMap<String, Command> eventCommandMap) {
        this.name = name;
        this.eventCommandMap = eventCommandMap;
        if (eventCommandMap.containsKey(name)){
            this.command = eventCommandMap.get(name);
        } else {
            this.command = null;
        }
    }

    @Override
    public void run() {
        // TODO Auto-generated method stub
        if (command != null){
            if (!command.isDone()) {
                command.run();
            } else {
                isDoneCommand = true;
            }
        }
    }

    @Override
    public boolean isDone() {
        // TODO Auto-generated method stub
        return isDoneCommand;
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        isDoneCommand = false;
    }
    
}
