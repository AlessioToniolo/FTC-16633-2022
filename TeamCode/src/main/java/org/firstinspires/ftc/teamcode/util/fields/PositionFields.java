package org.firstinspires.ftc.teamcode.util.fields;

public class PositionFields {
    // Slider Positions
    public static final int LOW = 200;
    public static final int MIDDLE = 750;
    public static final int TOP = 2000;

    // Intake Bar Positions
    public final double BUCKET_OVER = .35;
    public final double BUCKET_NOT_OVER = .475;

    // Bucket Positions
    public static final double BUCKET_INTAKE = .08;
    public static final double HOLDING = .35;
    public static final double OUTTAKE = 1;

    // Capstone Positions
    public static final double CAPSTONE_REST = .9;
    public static final double CAPSTONE_INCHES = .24;
    public static final double CAPSTONE_CAPPING = .6;

    // Speed Values
    public static final double MAX_CAROUSEL_SPEED = .8;
    public static final double STOP = 1.5;
    public static final double GO = 1.5;

    // Intake Timed Cycle Variable
    public static final double INTAKE_IN = 2;
    public static final double INTAKE_OUT = 1;

    // Modify the turning
    public static final double TURN_MOD = 4.25;

    // For Encoder Functions
    public static final double COUNTS_PER_MOTOR_REV = 537.6;
    public static final double DRIVE_GEAR_REDUCTION = 1.0; // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4; // For figuring out circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV / 360;
    public static final double DRIVE_SPEED = 1.0;
    public static final double TURN_SPEED = 1.0;
}
