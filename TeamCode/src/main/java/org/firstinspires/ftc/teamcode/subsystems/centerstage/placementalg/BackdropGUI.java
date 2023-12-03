package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop.COLUMNS;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop.ROWS;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Pixel.Color.SPACER;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PlacementCalculator.calculate;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Config
public final class BackdropGUI {

    public static double flashingTime = 0.5;

    private final Backdrop backdrop = new Backdrop();
    private ArrayList<Pixel> pixelsToPlace = new ArrayList<>();

    private Pixel selectedPixel = backdrop.get(1, 0);
    private boolean showSelection = true;
    private final ElapsedTime timer = new ElapsedTime();

    public BackdropGUI() {
        backdrop.printRectangular = false;
    }

    public void incrementY() {
        int newY = selectedPixel.y + 1;
        selectedPixel = backdrop.get(selectedPixel.x, (newY >= ROWS) ? 0 : newY);
        if (selectedPixel.x == 0 && selectedPixel.y % 2 == 0) {
            selectedPixel = backdrop.get(selectedPixel.x + 1, selectedPixel.y);
        }
    }

    public void decrementY() {
        int newY = selectedPixel.y - 1;
        selectedPixel = backdrop.get(selectedPixel.x, (newY < 0) ? ROWS - 1 : newY);
        if (selectedPixel.x == 0 && selectedPixel.y % 2 == 0) {
            selectedPixel = backdrop.get(selectedPixel.x + 1, selectedPixel.y);
        }
    }

    public void incrementX() {
        int newX = selectedPixel.x + 1;
        selectedPixel = backdrop.get((newX >= COLUMNS) ? (selectedPixel.y % 2 == 0 ? 1 : 0) : newX, selectedPixel.y);
    }

    public void decrementX() {
        int newX = selectedPixel.x - 1;
        selectedPixel = backdrop.get((newX < (selectedPixel.y % 2 == 0 ? 1 : 0)) ? COLUMNS - 1 : newX, selectedPixel.y);
    }

    public void flipColor() {
        int newColorInd = selectedPixel.color.ordinal() + 1;
        if (newColorInd > 4) newColorInd = 0;

        backdrop.add(new Pixel(selectedPixel, Pixel.Color.get(newColorInd)));
        selectedPixel = backdrop.get(selectedPixel.x, selectedPixel.y);
        pixelsToPlace = calculate(backdrop);
    }

    public void toTelemetry(MultipleTelemetry mTelemetry) {

        Pixel saved = selectedPixel;
        if (!showSelection) backdrop.add(new Pixel(selectedPixel, SPACER));

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

        Pixel saved = selectedPixel;
        if (!showSelection) backdrop.add(new Pixel(selectedPixel, SPACER));

        String[] rows = backdrop.toString().split("\n");

        for (String row : rows) System.out.println(row);
        System.out.println();
        for (Pixel pixel : pixelsToPlace) System.out.println(pixel.toString());

        if (!showSelection) backdrop.add(saved);

        if (timer.seconds() >= flashingTime) {
            showSelection = !showSelection;
            timer.reset();
        }
    }
}
