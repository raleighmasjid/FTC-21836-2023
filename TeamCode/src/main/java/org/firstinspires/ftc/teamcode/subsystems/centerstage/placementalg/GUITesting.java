package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static java.lang.Thread.sleep;

import java.util.Objects;
import java.util.Scanner;

final class GUITesting {

    public static void main(String[] args) throws InterruptedException {
        BackdropGUI backdropGUI = new BackdropGUI();
        Scanner input = new Scanner(System.in);

        while (true) {

            String key = input.nextLine();

            if (Objects.equals(key, "e")) backdropGUI.incrementY();
            else if (Objects.equals(key, "f")) backdropGUI.incrementX();
            else if (Objects.equals(key, "s")) backdropGUI.decrementX();
            else if (Objects.equals(key, "d")) backdropGUI.decrementY();
            else if (Objects.equals(key, "c")) backdropGUI.flipColor();

            backdropGUI.print();

            sleep(250);
        }
    }
}
