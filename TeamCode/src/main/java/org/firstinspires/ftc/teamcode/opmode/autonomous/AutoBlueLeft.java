package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.BaseRobot;
import org.firstinspires.ftc.teamcode.util.helpers.Printer;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous
public class AutoBlueLeft extends LinearOpMode {
    // Instance of Robot Class
    BaseRobot robot = new BaseRobot();

    // todo TESTING DISTANCE
    public static double distanceToMove = 1;

    // Delay ElapsedTime
    private ElapsedTime runtime = new ElapsedTime();

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

    // Inner Class for Pipeline
    class Pipeline_Target_Detect extends OpenCvPipeline {
        public Mat templateMat = new Mat(64, 32, CvType.CV_8UC1);

        public Pipeline_Target_Detect() {
            buildTemplate();
        }

        @Override
        public Mat processFrame(Mat input) {
            Mat gray = new Mat();
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
            Mat heatMap = new Mat();
            //normalized- TM_COEFF
            //SSD - TM_SQDIFF
            int machMethod = Imgproc.TM_SQDIFF;//gets the int representing this method from the Imgproc class
            Imgproc.matchTemplate(gray, templateMat, heatMap, machMethod);//tells it to make a heat map using SSD(Sum of Squared Differences)

            Core.MinMaxLocResult mmr = Core.minMaxLoc(heatMap);
            Point matchLoc = mmr.minLoc;
            xPos = (double) matchLoc.x;

            int pointX = (int) matchLoc.x;
            int pointY = (int) matchLoc.y;

            //value = heatMap.get(pointY, pointX)[0];
            Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x + templateMat.cols(), matchLoc.y + templateMat.rows()), new Scalar(0, 255, 0));

            //return input;
            return input;
        }

        private void buildTemplate() {
            int[][] template = new int[64][32];
            for (int r = 0; r < 32; r++) {
                for (int c = 0; c < 32; c++) {//0 black 255 is white
                    template[r][c] = 255;
                }
            }
            for (int r = 32; r < 64; r++) {
                for (int c = 0; c < 32; c++) {
                    template[r][c] = 0;
                }
            }
            for (int r = 0; r < 64; r++) {
                for (int c = 0; c < 32; c++) {
                    templateMat.put(r, c, template[r][c]);
                }
            }
        }
    }

    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            //telemetry.addData("Time: ", runtime.seconds());
            //telemetry.update();
        }
    }
}
