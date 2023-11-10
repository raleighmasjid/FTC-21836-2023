package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.List;

@Autonomous(group = "21836 Autonomous", preselectTeleOp = "MainTeleOp")
public class MainAuton extends LinearOpMode {

    // Declare objects:
    MultipleTelemetry myTelemetry;
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        myTelemetry = new MultipleTelemetry(telemetry);

        // Initialize internal hub representations:
        // Switch hubs to manually reset sensor inputs when we tell it to:
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            for (LynxModule hub : hubs) hub.clearBulkCache();

            // Push telemetry data to multiple outputs (set earlier):
            myTelemetry.update();
        }
    }
}
