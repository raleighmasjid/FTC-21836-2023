package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.GREEN;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.PURPLE;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.WHITE;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.YELLOW;

import java.util.Objects;
import java.util.Scanner;

final class GUITesting {

    public static void main(String[] args) {
        BackdropGUI backdropGUI = new BackdropGUI();
        Scanner input = new Scanner(System.in);

        while (true) {

            String key = input.nextLine();

            if (Objects.equals(key, "e")) backdropGUI.incrementY();
            else if (Objects.equals(key, "f")) backdropGUI.incrementX();
            else if (Objects.equals(key, "s")) backdropGUI.decrementX();
            else if (Objects.equals(key, "d")) backdropGUI.decrementY();
            else if (Objects.equals(key, "z")) backdropGUI.changeTo(EMPTY);
            else if (Objects.equals(key, "x")) backdropGUI.changeTo(WHITE);
            else if (Objects.equals(key, "c")) backdropGUI.changeTo(PURPLE);
            else if (Objects.equals(key, "v")) backdropGUI.changeTo(YELLOW);
            else if (Objects.equals(key, "b")) backdropGUI.changeTo(GREEN);

            backdropGUI.print();
        }
    }
}
