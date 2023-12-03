package org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop.COLUMNS;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.Backdrop.ROWS;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.placementalg.PlacementCalculator.calculate;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public final class BackdropGUI {

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
        selectedPixel = backdrop.get((newX >= COLUMNS) ? 0 : (selectedPixel.y % 2 == 0 ? 1 : 0), selectedPixel.y);
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
        String[] rows = backdrop.toString().split("\n");

        for (String row : rows) mTelemetry.addLine(row);
        mTelemetry.addLine();
        for (Pixel pixel : pixelsToPlace) mTelemetry.addLine(pixel.toString());
    }

    public void print() {
        String[] rows = backdrop.toString().split("\n");

        if (!showSelection) {
            String row = rows[10 - selectedPixel.y];
            int xIndex = selectedPixel.x + (selectedPixel.y % 2 == 0 ? 3 : 4);
            rows[10 - selectedPixel.y] = row.substring(0, xIndex) + " " + row.substring(xIndex + (selectedPixel.color.isColored() ? 6 : 1));
        }

        for (String row : rows) System.out.println(row);
        System.out.println();
        for (Pixel pixel : pixelsToPlace) System.out.println(pixel.toString());

        if (timer.seconds() >= 0.5) {
            showSelection = !showSelection;
            timer.reset();
        }
    }
}
