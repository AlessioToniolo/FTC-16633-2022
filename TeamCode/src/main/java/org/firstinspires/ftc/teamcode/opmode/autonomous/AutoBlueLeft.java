package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.BaseRobot;
import org.firstinspires.ftc.teamcode.util.helpers.Printer;

@Autonomous
public class AutoBlueLeft extends LinearOpMode {
    // Instance of Robot Class
    BaseRobot robot = new BaseRobot();

    // Instance of a Printing Class for Telemetry
    Printer printer = new Printer();

    @Override
    public void runOpMode() throws InterruptedException {
        // TODO: testing error
        //printer.print("Wait for Start!");
        printer.prit("Wait for Start!");

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

    // Functions for the different zones
    private void zone1() {
    }

    private void zone2() {
    }

    private void zone3() {
    }

}
