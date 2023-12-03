package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop.COLUMNS;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop.ROWS;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.EMPTY;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.HIGHLIGHTED;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.INVALID;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PlacementCalculator.calculate;
import static java.lang.System.out;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Config
public final class BackdropGUI {

    public static double flashingTime = 0.5;

    private final Backdrop backdrop = new Backdrop();
    private ArrayList<Pixel> pixelsToPlace = calculate(backdrop);

    private Pixel selectedPixel;
    private boolean showSelection = true;
    private final ElapsedTime timer = new ElapsedTime();

    public BackdropGUI() {
        backdrop.printRectangular = false;

        for (int i = 0; i < 15; i++) {
            backdrop.add(pixelsToPlace.get(0));
            pixelsToPlace = calculate(backdrop);
        }
        selectedPixel = backdrop.get(1, 0);
    }

    public void up() {
        int newY = selectedPixel.y + 1;
        selectedPixel = backdrop.get(selectedPixel.x, (newY >= ROWS) ? 0 : newY);
        if (selectedPixel.x == 0 && selectedPixel.y % 2 == 0) {
            selectedPixel = backdrop.get(selectedPixel.x + 1, selectedPixel.y);
        }
    }

    public void down() {
        int newY = selectedPixel.y - 1;
        selectedPixel = backdrop.get(selectedPixel.x, (newY < 0) ? ROWS - 1 : newY);
        if (selectedPixel.x == 0 && selectedPixel.y % 2 == 0) {
            selectedPixel = backdrop.get(selectedPixel.x + 1, selectedPixel.y);
        }
    }

    public void right() {
        int newX = selectedPixel.x + 1;
        selectedPixel = backdrop.get((newX >= COLUMNS) ? (selectedPixel.y % 2 == 0 ? 1 : 0) : newX, selectedPixel.y);
    }

    public void left() {
        int newX = selectedPixel.x - 1;
        selectedPixel = backdrop.get((newX < (selectedPixel.y % 2 == 0 ? 1 : 0)) ? COLUMNS - 1 : newX, selectedPixel.y);
    }

    public void update(Pixel.Color color) {
        backdrop.add(new Pixel(selectedPixel, color));
        selectedPixel = backdrop.get(selectedPixel.x, selectedPixel.y);
        pixelsToPlace = calculate(backdrop);
    }

    public void toTelemetry(MultipleTelemetry mTelemetry) {
        Pixel saved = selectedPixel;
        if (!showSelection) backdrop.add(new Pixel(selectedPixel, INVALID));

        String[] rows = backdrop.toString().split("\n");

        for (String row : rows) mTelemetry.addLine(row);
        mTelemetry.addLine();
        for (Pixel pixel : pixelsToPlace) mTelemetry.addLine(pixel.toString());

        if (!showSelection) backdrop.add(saved);

        if (timer.seconds() >= flashingTime) {
            showSelection = !showSelection;
            timer.reset();
        }
    }

    public void print() {

        Pixel toFill = new Pixel(pixelsToPlace.get(0), EMPTY);
        backdrop.add(new Pixel(toFill, HIGHLIGHTED));

        Pixel saved = selectedPixel;
        if (!showSelection) backdrop.add(new Pixel(saved, INVALID));

        String[] rows = backdrop.toString().split("\n");
        for (String row : rows) out.println(row);
        out.println();
        out.println(HIGHLIGHTED + " " + pixelsToPlace.get(0).color.name());
        for (Pixel pixel : pixelsToPlace) out.println(pixel.toString());

        if (!showSelection) backdrop.add(saved);

        if (timer.seconds() >= flashingTime) {
            showSelection = !showSelection;
            timer.reset();
        }

        backdrop.add(toFill);
    }
}
