package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Sensor Test TeleOp for beam break sensors.
 * 
 * This program reads three digital beam break sensors on ports 0, 1, and 2
 * and displays whether each beam is broken or not.
 * 
 * Beam break sensors typically return:
 * - false when the beam is BROKEN (object present)
 * - true when the beam is NOT BROKEN (clear)
 */
@Disabled
@TeleOp(name = "Sensor Test", group = "Test")
public class SensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Get the three digital sensors
        DigitalChannel sensor0 = hardwareMap.get(DigitalChannel.class, "sensor0");
        DigitalChannel sensor1 = hardwareMap.get(DigitalChannel.class, "sensor1");
        DigitalChannel sensor2 = hardwareMap.get(DigitalChannel.class, "sensor2");

        // Set all sensors to input mode
        sensor0.setMode(DigitalChannel.Mode.INPUT);
        sensor1.setMode(DigitalChannel.Mode.INPUT);
        sensor2.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("Sensor Test Ready");
        telemetry.addLine("Reading beam break sensors on digital ports 0, 1, 2");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Read sensor states
            // Note: beam break sensors typically return false when broken
            boolean state0 = sensor0.getState();
            boolean state1 = sensor1.getState();
            boolean state2 = sensor2.getState();

            // Display sensor states
            telemetry.addData("Sensor 0 (Port 0)", state0 ? "NOT BROKEN" : "BROKEN");
            telemetry.addData("Sensor 1 (Port 1)", state1 ? "NOT BROKEN" : "BROKEN");
            telemetry.addData("Sensor 2 (Port 2)", state2 ? "NOT BROKEN" : "BROKEN");
            
            telemetry.addLine();
            telemetry.addLine("Raw values (for debugging):");
            telemetry.addData("Sensor 0 raw", state0);
            telemetry.addData("Sensor 1 raw", state1);
            telemetry.addData("Sensor 2 raw", state2);
            
            telemetry.update();

            idle();
        }
    }
}

