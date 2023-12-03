package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static java.lang.Thread.sleep;

final class GUITesting {

    public static void main(String[] args) throws InterruptedException {
        BackdropGUI backdropGUI = new BackdropGUI();

        boolean lastUp = false, lastDown = false, lastRight = false, lastLeft = false;


        while (true) {
            backdropGUI.flipColor();
            backdropGUI.print();

            sleep(250);
        }
    }
}
