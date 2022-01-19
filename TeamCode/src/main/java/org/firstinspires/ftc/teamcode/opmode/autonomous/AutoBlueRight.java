package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BaseRobot;
import org.firstinspires.ftc.teamcode.util.helpers.Printer;

@Autonomous
public class AutoBlueRight extends LinearOpMode {
    // Instance of Robot Class
    BaseRobot robot = new BaseRobot();

    // Instance of a Printing Class for Telemetry
    Printer printer;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry
        printer = new Printer(telemetry);

        printer.print("Wait for Start!");

        // Initialize Hardware
        robot.init(hardwareMap, true);

        // Signal that robot is ready to run
        printer.print("Ready to start!");

        // Wait for User to Start
        waitForStart();

        // TODO: movements
    }

    // Main Function that runs before the zone functions
    private void mainMovement() {

    }


}
