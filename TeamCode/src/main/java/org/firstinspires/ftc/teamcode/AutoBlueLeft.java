package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.BaseRobot;
import org.firstinspires.ftc.teamcode.util.Printer;

public class AutoBlueLeft extends LinearOpMode {
    // Instance of Robot Class
    BaseRobot robot = new BaseRobot();

    // Instance of a Printing Class for Telemetry
    Printer printer = new Printer();

    @Override
    public void runOpMode() throws InterruptedException {
        printer.print("Wait for Start!");

        // Initialize Hardware
        robot.init(hardwareMap);

        // Signal that robot is ready to run
        printer.print("Ready to start!");

        // Wait for User to Start
        waitForStart();
    }
}
