package org.firstinspires.ftc.teamcode.opmode.autonomous;
import org.firstinspires.ftc.teamcode.opmode.autonomous.opencvpipeline.Pipeline_Target_Detect;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.BaseRobot;
import org.firstinspires.ftc.teamcode.util.helpers.Printer;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Config
@Autonomous
public class AutoBlueLeft extends LinearOpMode {
    // Instance of Robot Class
    BaseRobot robot = new BaseRobot();

    // todo TESTING DISTANCE
    public static double distanceToMove = 1;

    // Delay ElapsedTime
    private final ElapsedTime runtime = new ElapsedTime();

    // OpenCV
    WebcamName webcamName;
    OpenCvCamera camera;
    Pipeline_Target_Detect myPipeline;
    double zone = 3;
    double xPos = -1;

    // Instance of a Printing Class for Telemetry
    Printer printer;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry
        printer = new Printer(telemetry);

        // Tell User to Wait For Start (to allow motors to instantiate)
        printer.print("Wait for Start!");

        // Initialize Hardware
        robot.init(hardwareMap, true);

        // Signal that robot is ready to run
        printer.print("Ready to start!");

        // Wait for User to Start
        waitForStart();

        // Clear Previous Telemetry
        printer.load();

        // Init Camera
        // TODO change webcam to webcam 3 once it is installed
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        myPipeline = new Pipeline_Target_Detect();
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(myPipeline);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        // Wait for Camera to Start Up and Detect
        delay(2);
        xPos = myPipeline.getXPos();
        // Read Detection
        if (xPos < 100) {
            zone = 1;
        } else if (xPos > 200) {
            zone = 3;
        } else {
            zone = 2;
        }

        // Close Camera
        camera.stopStreaming();
        camera.closeCameraDevice();

        // Print Detection to Drivers
        printer.addInfo("XPos: ", xPos);
        printer.addInfo("Zone: ", zone);
        printer.load();

        // TODO: movements
        robot.move(distanceToMove, 4);
        robot.turn(180, 4);
    }

    // Main Function that runs before the zone functions
    private void mainMovement() {
        // TODO going to the hub movement
        if (zone == 1) {
            zone1();
        } else if (zone == 2) {
            zone2();
        } else {
            zone3();
        }
        // TODO rest of movements
    }

    // Functions for the different zones
    private void zone1() {
        // TODO
    }

    private void zone2() {
        // TODO
    }

    private void zone3() {
        // TODO
    }

    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            //telemetry.addData("Time: ", runtime.seconds());
            //telemetry.update();
        }
    }
}
