package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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

    // Local OpMode members
    HardwareMap hwMap;

    //IMU Fields
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    BNO055IMU.Parameters imuParameters;
    double theta;
    double previousHeading = 0;
    double integratedHeading = 0;
    private ElapsedTime period = new ElapsedTime();
    boolean isTurning = false;

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

