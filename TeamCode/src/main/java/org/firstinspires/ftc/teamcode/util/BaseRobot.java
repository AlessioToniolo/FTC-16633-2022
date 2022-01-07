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
import org.firstinspires.ftc.teamcode.util.fields.PositionFields;

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
    BNO055IMU.Parameters imuParameters;
    double previousHeading = 0;
    double integratedHeading = 0;
    private ElapsedTime period = new ElapsedTime();

    // Constructor - leave this blank
    public BaseRobot() {
    }

    // Initialize Standard Hardware Interfaces
    public void init(HardwareMap ahwMap, boolean RUN_USING_ENCODERS) {
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

        // Option for run without encoders (teleop) or rue (auto)
        if (RUN_USING_ENCODERS) {
            // Set all motors to run using encoder
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            // Set all motors to run without encoders
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    // Move forward and backward using a PID loop and motor encoders
    public void move(double distance, double timeoutS) {
        // p stands for proportional
        double p;
        // d stands for derivative and gets the current rate of change
        double d;
        // power adds p and d;
        double power;
        double currentError = 0;
        double previousError = 0;

        // time for loop
        ElapsedTime distanceTime;

        // Get the first average of encoder counts
        double offset = (double) ((leftFront.getCurrentPosition() + leftRear.getCurrentPosition()
                + rightFront.getCurrentPosition() + rightRear.getCurrentPosition()) / 4);
        // Adapt target to curr position
        double target = PositionFields.COUNTS_PER_INCH * distance + offset;

        // While
        if (distance > 0) {
            distanceTime = new ElapsedTime();
            while (distanceTime.seconds() < timeoutS) {
                // Compute the current error
                currentError = target - (double) ((leftFront.getCurrentPosition() + leftRear.getCurrentPosition()
                        + rightFront.getCurrentPosition() + rightRear.getCurrentPosition()) / 4);

                // Compute
                p = PIDFields.MOVE_PID.p * currentError;
                d = PIDFields.MOVE_PID.d * (currentError - previousError);

                // Add the gains
                power = p + d;

                // move forward
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftRear.setPower(-power);
                rightRear.setPower(-power);

                previousError = currentError;
            }
            distanceTime = null;
        } else {
            distanceTime = new ElapsedTime();
            while (distanceTime.seconds() < timeoutS) {
                // Compute the current error
                currentError = target - (double) ((leftFront.getCurrentPosition() + leftRear.getCurrentPosition()
                        + rightFront.getCurrentPosition() + rightRear.getCurrentPosition()) / 4);

                // Compute
                p = PIDFields.MOVE_PID.p * currentError;
                d = PIDFields.IMU_TURN_PID.d * (currentError - previousError);

                // Add the gains
                power = p + d;

                // move backward
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftRear.setPower(power);
                rightRear.setPower(power);

                previousError = currentError;
            }
            distanceTime = null;
        }

        // Turn off motors once done
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    // Move forward and backward using a PID loop and motor encoders along with a heading PID from the IMU
    public void headingMove(double distance, double timeoutS) {
        // 1 is axial and 2 is heading
        // p stands for proportional
        double p1;
        double p2;
        // d stands for derivative and gets the current rate of change
        double d1;
        double d2;
        // power adds p and d;
        double power1;
        double power2;
        double currentError1 = 0;
        double previousError1 = 0;
        double currentError2 = 0;
        double previousError2 = 0;

        // time for loop
        ElapsedTime distanceTime;

        // Get the first average of encoder counts
        double offset1 = (double) ((leftFront.getCurrentPosition() + leftRear.getCurrentPosition()
                + rightFront.getCurrentPosition() + rightRear.getCurrentPosition()) / 4);
        double offset2 = getIntegratedHeading();
        // Adapt target to curr position
        double target1 = PositionFields.COUNTS_PER_INCH * distance + offset1;
        double target2 = offset2;

        // While
        if (distance > 0) {
            distanceTime = new ElapsedTime();
            while (distanceTime.seconds() < timeoutS) {
                // Compute the current error
                currentError1 = target1 - (double) ((leftFront.getCurrentPosition() + leftRear.getCurrentPosition()
                        + rightFront.getCurrentPosition() + rightRear.getCurrentPosition()) / 4);

                // Compute
                p1 = PIDFields.MOVE_PID.p * currentError1;
                d1 = PIDFields.MOVE_PID.d * (currentError1 - previousError1);

                // Add the gains
                power1 = p1 + d1;
                power2 = p1 + d1;

                // Heading
                // Compute heading currrent error
                currentError2 = target2 - getIntegratedHeading();

                // Compute
                p2 = PIDFields.IMU_TURN_PID.p * currentError2;
                d2 = PIDFields.IMU_TURN_PID.d * currentError2;

                // Compute Heading Into Power
                // TODO: this would probably be the biggest source of a problem if it does not work
                if (currentError2 > 0) {
                    power1 = power1 - (p2 + d2);
                } else {
                    power2 = power2 - (p2 + d2);
                }

                // move forward
                leftFront.setPower(-power1);
                rightFront.setPower(-power2);
                leftRear.setPower(-power1);
                rightRear.setPower(-power2);

                previousError1 = currentError1;
                previousError2 = currentError2;
            }
            distanceTime = null;
        } else {
            distanceTime = new ElapsedTime();
            while (distanceTime.seconds() < timeoutS) {
                // Compute the current error
                currentError1 = target1 - (double) ((leftFront.getCurrentPosition() + leftRear.getCurrentPosition()
                        + rightFront.getCurrentPosition() + rightRear.getCurrentPosition()) / 4);

                // Compute
                p1 = PIDFields.MOVE_PID.p * currentError1;
                d1 = PIDFields.MOVE_PID.d * (currentError1 - previousError1);

                // Add the gains
                power1 = p1 + d1;
                power2 = p1 + d1;

                // Heading
                // Compute heading currrent error
                currentError2 = target2 - getIntegratedHeading();

                // Compute
                p2 = PIDFields.IMU_TURN_PID.p * currentError2;
                d2 = PIDFields.IMU_TURN_PID.d * currentError2;

                // Compute Heading Into Power
                // TODO: this would probably be the biggest source of a problem if it does not work
                if (currentError2 > 0) {
                    power1 = power1 - (p2 + d2);
                } else {
                    power2 = power2 - (p2 + d2);
                }

                // move forward
                leftFront.setPower(power1);
                rightFront.setPower(power2);
                leftRear.setPower(power1);
                rightRear.setPower(power2);

                previousError1 = currentError1;
                previousError2 = currentError2;
            }
            distanceTime = null;
        }

        // Turn off motors once done
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
    }

    // Turn Using the IMU and PID
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

    // Get Current IMU Heading that is adapted to a Cumulative Scale
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

