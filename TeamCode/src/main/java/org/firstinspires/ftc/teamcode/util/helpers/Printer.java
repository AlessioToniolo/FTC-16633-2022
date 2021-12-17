package org.firstinspires.ftc.teamcode.util.helpers;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class Printer {
    // Constructor
    public Printer() {
    }

    // Function for easy telemetry printing without update
    public void message(String message) {
        telemetry.addLine(message);
    }

    // Function for easy telemetry printing with update
    public void print(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }

    // To print multiple lines
    public void printLines(ArrayList<String> str) {
        for (int i = 0; i < str.size(); i++) {
            message(str.get(i));
        }
        telemetry.update();
    }
}
