package org.firstinspires.ftc.teamcode.subsystems.utilities.threads;

public class ThreadedExecution extends java.lang.Thread {

    public interface Command {
        void execute();
    }

    public static void executeCommands(Command... commands) {
        for (Command command : commands) command.execute();
    }

    private final Command[] commands;

    public ThreadedExecution(Command... commands) {
        this.commands = commands;
        start();
    }

    public void run() {
        executeCommands(commands);
        Thread.yield();
    }
}

