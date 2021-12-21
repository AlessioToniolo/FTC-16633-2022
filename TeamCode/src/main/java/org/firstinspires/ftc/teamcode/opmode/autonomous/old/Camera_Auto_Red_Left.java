package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.CvType;
import org.opencv.core.Core;
import org.opencv.core.Core.MinMaxLocResult;
import org.opencv.core.CvType;
import org.opencv.core.Core;
import org.opencv.core.Core.MinMaxLocResult;

@Autonomous
public class Camera_Auto_Red_Left extends LinearOpMode {

    /* Declare OpMode members. */
    MaristBaseRobot2021_Quad robot   = new MaristBaseRobot2021_Quad();   
    private ElapsedTime runtime = new ElapsedTime();
    public  final int bottom = robot.low;
    public  final int middle = robot.middle;
    public  final int top = robot.top;
    // Variables to control Speed
    double velocity = 0.5; // Default velocity

    WebcamName webcamName;
    OpenCvCamera camera;
    
    Pipeline_Target_Detect myPipeline;
    int zone=3;
    double xPos = -1;
    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        
        robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status: ", "Ready to run");    //
        telemetry.update();
        
        
        waitForStart();
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        myPipeline = new Pipeline_Target_Detect();
        
        
        
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                
                camera.setPipeline(myPipeline);
            }
            @Override
            public void onError(int errorCode)
            {
                
            }
        });
        
        
        telemetry.addData("Waiting for Camera", "5 Seconds");
        telemetry.update();
        delay(2);
        
        //double value = -1;
        runtime.reset();
       //while(opModeIsActive() && runtime.seconds() < 1) {
        //    //xPos = myPipeline.getXPos();
         //value = myPipeline.getValue();
        //    telemetry.addData("xPos: ", xPos);
        //    telemetry.update();
        //}
        
        
        
        
        if(xPos < 100)
        {
            zone = 1;
        }
        else if(xPos > 200)
        {
            zone = 3;
        }
        else 
        {
            zone = 2;
        }
        
        telemetry.addData("XPos: ", xPos);
        telemetry.addData("Zone: ", zone);
        telemetry.update();
        
        //delay(10);
        
        //camera.release();
        camera.stopStreaming();
        camera.closeCameraDevice();
         forward(-20);//goes to hub
        if (zone == 1) {
            robot.slider.setTargetPosition(bottom);
            robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slider.setPower(1);
            delay(2);
        } else if (zone == 2) {
            robot.slider.setTargetPosition(middle);
            robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slider.setPower(1);
            delay(2);
        } else {
            robot.slider.setTargetPosition(top);
            robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.slider.setPower(1);
            delay(2);        }
        
        // RUN CODE HERE
        
        robot.bucket.setPosition(.85);
        delay(2);
        robot.bucket.setPosition(.08);
        delay(1);
        robot.slider.setTargetPosition(0);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(1);
        forward(20);//returns to strting position
        velocity = .5;//slows down to straighten itself againt the wall
        forward(15);
        velocity = 1;
        forward(-12);//go forward
        delay(1);
        left(-90);
        delay(0.5);
        forward(30);
        velocity = .5;
        forward(15);
        delay(0.5);
        forward(-5);
        delay(0.5);
        velocity = 1;
        left(72);
        delay(0.5);
        forward(13);
        robot.carousel.setPower(-0.6);
        delay(3);
        robot.carousel.setPower(0);
        forward(-13);
        //forward(5);
        //right(90);
        //forward(15);
        /*forward(-35);// go to the carousel area
        delay(0.5);
        right(90); // turn to carousel
        delay(0.5);
        forward(11);// go to carousel
        robot.carousel.setPower(-0.6);
        delay(3);
        robot.carousel.setPower(0);
        delay(0.5);
        // go to park
        forward(-18);*/
        
        
        
        telemetry.addData("XPos: ", xPos);
        telemetry.addData("Zone: ", zone);
        telemetry.update();
        delay(3);
        //camera.stopStreaming();
        //camera.closeCameraDevice();
        // Autonomous Finished
        telemetry.addData("Path", "Complete");
        telemetry.update();
        //sleep(1000);
    }








    // Functions for REACH 2019 based on Python Turtles
    public void forward(double inches)
    {
        robot.driveStraightInches(velocity, inches, 10);
    }
    
    public void right(double degrees)
    {
        degrees=degrees*robot.turnMod;
        robot.pointTurnDegrees(velocity, degrees, 10);
    }
    
    public void left(double degrees)
    {
        degrees = degrees * -1*robot.turnMod;
        robot.pointTurnDegrees(velocity, degrees, 10);
    }
    
    public void speed(int speed)
    {
        double newSpeed = (double)speed / 10.0;
        velocity = newSpeed;
    }
    
    // Sample Delay Code
    public void delay(double t) { // Imitates the Arduino delay function
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            //telemetry.addData("Time: ", runtime.seconds());
            //telemetry.update();
        }
    }
    
    
    // Inner Class for Pipeline
    
    class Pipeline_Target_Detect extends OpenCvPipeline
    {
        public Mat templateMat = new Mat(64, 32, CvType.CV_8UC1);
    
        //public double xPos = -1;
        //public double value = -1;
       
       
       
        public Pipeline_Target_Detect()
        {
            buildTemplate();
        }
        @Override
        public Mat processFrame(Mat input)
        {
           
            Mat gray = new Mat();
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
            Mat heatMap = new Mat();
            int machMethod = Imgproc.TM_CCOEFF;
            Imgproc.matchTemplate(gray, templateMat, heatMap, machMethod);
           
            MinMaxLocResult mmr = Core.minMaxLoc(heatMap);
            Point matchLoc = mmr.maxLoc;
            xPos = (double)matchLoc.x;
            
            int pointX = (int)matchLoc.x;
            int pointY = (int)matchLoc.y;
            
             //value = heatMap.get(pointY, pointX)[0];
            Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x+templateMat.cols(),matchLoc.y + templateMat.rows()), new Scalar(0, 255, 0));
           
            //return input;
            return templateMat; 
        }
         private void buildTemplate()
    {
        int[][] template = new int[64][32];
        for(int r = 0; r < 32; r++)
        {
            for(int c = 0; c < 32; c++)
            {//0 black 255 is white
                template[r][c] = 255;
            }
        }
        for(int r = 32; r < 64; r++)
        {
            for(int c =0; c < 32; c++)
            {
                template[r][c] = 0;
            }
        }
       
       
        for(int r = 0; r < 64; r++)
        {
            for(int c = 0; c < 32; c++)
            {
                templateMat.put(r, c, template[r][c]);
            }
        }
    }
    

    // todo: write your code here
    }

    
    
   
}

