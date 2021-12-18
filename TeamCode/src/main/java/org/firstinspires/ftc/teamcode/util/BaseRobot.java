package org.firstinspires.ftc.teamcode.util;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.fields.PIDFields;

public class BaseRobot {
    // Drive Motors
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftRear;
    public DcMotor rightRear;
    // Intake Motor
    public DcMotor intake;
    // Carousel Motor
    public DcMotor carousel;
    // Linear Slider Motor
    public DcMotor slider;
    // Linear Slider Deposit Bucket Servo
    public Servo bucket;
    public Servo intakeBar;
    public Servo capstoneArm;


    //FINAL INTS nts for auto and teleop
    //MotorEncoder positions
    //Slider Positions
    public final int low = 400;
    public final int middle = 1050;
    public final int top = 1800;
    //Carousel Motor ints
    //max carousel speed before starts to spin ducks off
    public final double maxcarouselspeed =  1;
    public final double stop = 1.5;
    public final double go = 1.5;

    //SERVO POSITIONS
    //intake bar positions
    public final double bucketover = .35;
    public final double bucketnotover = .475;
    //bucket positions
    public final double bucketintake = .08;
    public final double holding = .35;
    public final double outtake = 1;
    //capstone positions
    public final double capstonerest = .9;
    public final double capstoneintake = .24;
    public final double capstonecapping = .6;

    // For Encoder Functions
    private double     COUNTS_PER_MOTOR_REV          = 537.6 ;    // eg: TETRIX Motor Encoder
    private final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    private double     WHEEL_DIAMETER_INCHES         = 4 ;     // For figuring circumference
    private double     COUNTS_PER_INCH               = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private double COUNTS_PER_DEGREE                 = COUNTS_PER_MOTOR_REV / 360;
    private double     DRIVE_SPEED                   = 1.0;
    private double     TURN_SPEED                    = 1.0;
    // Local OpMode members
    HardwareMap hwMap;

    //IMU Fields
    BNO055IMU imu;
    BNO055IMU.Parameters imuParameters;
    double previousHeading = 0;
    double integratedHeading = 0;
    private ElapsedTime period = new ElapsedTime();

    // Constructor - leave this blank
    public BaseRobot() {
    }

    // Initialize Standard Hardware Interfaces
    public void init(HardwareMap ahwMap) {
        // Save Reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors.  Assign Names that match the setup on the RC Phone
        leftFront = hwMap.dcMotor.get("leftfront");
        rightFront = hwMap.dcMotor.get("rightfront");
        leftRear = hwMap.dcMotor.get("leftrear");
        rightRear = hwMap.dcMotor.get("rightrear");
        carousel = hwMap.dcMotor.get("carousel");
        intake = hwMap.dcMotor.get("intake");
        slider = hwMap.dcMotor.get("slider");
        bucket = hwMap.servo.get("bucket");
        intakeBar = hwMap.servo.get("intakeBar");
        capstoneArm = hwMap.servo.get("capstone");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        carousel.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.FORWARD);

        // Initialize IMU
        imu = hwMap.get(BNO055IMU.class, "imu");
        imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu.initialize(imuParameters);

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        // Set all motors to run without encoders
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turn(double deg, double timeoutS) {
        // p stands for proportional
        double p;
        // d stands for derivative and gets the current rate of change
        double d;
        // power adds p and d;
        double power;
        double currentError = 0;
        double previousError = 0;

        // time for loop
        ElapsedTime imuTime;

        // Get the first heading
        double offset = getIntegratedHeading();
        // Adapt target to curr heading
        double imuTarget = deg + offset;

        // While
        if (deg > 0) {
            imuTime = new ElapsedTime();
            while (imuTime.seconds() < timeoutS) {
                // Compute the current error
                currentError = imuTarget - getIntegratedHeading();

                // Compute
                p = PIDFields.IMU_TURN_PID.p * currentError;
                d = PIDFields.IMU_TURN_PID.d * (currentError - previousError);

                // Add the gains
                power = p + d;

                // turn left
                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftRear.setPower(power);
                rightRear.setPower(-power);

                previousError = currentError;
            }
            imuTime = null;
        } else {
            imuTime = new ElapsedTime();
            while (imuTime.seconds() < timeoutS) {
                // Compute the current error
                currentError = imuTarget - getIntegratedHeading();

                // Compute
                p = PIDFields.IMU_TURN_PID.p * currentError;
                d = PIDFields.IMU_TURN_PID.d * (currentError - previousError);

                // Add the gains
                power = p + d;

                // turn left
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftRear.setPower(-power);
                rightRear.setPower(power);

                previousError = currentError;
            }
            imuTime = null;
        }

        // Turn off motors once done
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    private double getIntegratedHeading() {
        double currentHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double deltaHeading = currentHeading - previousHeading;

        if (deltaHeading < -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        integratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return integratedHeading;
    }
}

