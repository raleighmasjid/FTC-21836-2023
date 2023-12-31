package org.firstinspires.ftc.teamcode.subsystems.utilities.threads;

import static org.firstinspires.ftc.teamcode.subsystems.utilities.threads.ThreadedExecution.executeCommands;

public class ThreadedLoop {

    private volatile boolean run = true;

    public ThreadedLoop(ThreadedExecution.Command... commands) {
        new ThreadedExecution(() -> {
            while (run) {
                executeCommands(commands);
                Thread.yield();
            }
        });
    }

    public void endLoop() {
        run = false;
    }
}
