package org.firstinspires.ftc.teamcode.opmode.testerclasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.autonomous.opencvpipeline.Pipeline_Target_Detect;
import org.firstinspires.ftc.teamcode.util.BaseRobot;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class Camera_Tester extends OpMode {
    BaseRobot robot = new BaseRobot();
    WebcamName webcamName;
    OpenCvCamera camera;
    Pipeline_Target_Detect myPipeline;
    int zone = 3;
    double xPos = -1;
    int width = 64;
    boolean preValueLBumper = false;
    boolean preValueRBumper = false;
    boolean preValueA = false;
    boolean cameraOpen = false;

    public void init() {
    }

    public void loop() {
        if (gamepad1.left_bumper && gamepad1.left_bumper != preValueLBumper) {
            width -= 2;
        }
        preValueLBumper = gamepad1.left_bumper;

        if (gamepad1.right_bumper && gamepad1.right_bumper != preValueRBumper) {
            width += 2;
        }
        preValueRBumper = gamepad1.right_bumper;
        if (gamepad1.a && gamepad1.a != preValueA) {
            if(!cameraOpen) {//if the camera isnt open lets open it
                cameraOpen = true;
                webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
                camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
                myPipeline = new Pipeline_Target_Detect(width);
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
                xPos = myPipeline.getXPos();
                telemetry.addLine("XPOs: " + xPos);
                if (xPos < 100) {
                    zone = 1;
                } else if (xPos > 200) {
                    zone = 3;
                } else {
                    zone = 2;
                }
                telemetry.addLine("Zone: " + zone);
            }
            else if(cameraOpen) {
                cameraOpen = false;
                camera.stopStreaming();
                camera.closeCameraDevice();
            }
        }
        preValueA = gamepad1.a;
        telemetry.addLine("Width: " + width);
        telemetry.addLine("Camera OPen" + cameraOpen);
        telemetry.update();
    }


}
