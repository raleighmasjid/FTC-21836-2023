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

            if (Objects.equals(key, "e")) backdropGUI.up();
            else if (Objects.equals(key, "f")) backdropGUI.right();
            else if (Objects.equals(key, "s")) backdropGUI.left();
            else if (Objects.equals(key, "d")) backdropGUI.down();
            else if (Objects.equals(key, "z")) backdropGUI.update(EMPTY);
            else if (Objects.equals(key, "x")) backdropGUI.update(WHITE);
            else if (Objects.equals(key, "c")) backdropGUI.update(PURPLE);
            else if (Objects.equals(key, "v")) backdropGUI.update(YELLOW);
            else if (Objects.equals(key, "b")) backdropGUI.update(GREEN);

            backdropGUI.print();
        }
    }
}
