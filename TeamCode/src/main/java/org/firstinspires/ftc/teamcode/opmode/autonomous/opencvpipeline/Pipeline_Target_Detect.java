package org.firstinspires.ftc.teamcode.opmode.autonomous.opencvpipeline;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline_Target_Detect extends OpenCvPipeline {
    private int xPos =-1;
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
        xPos = (int) matchLoc.x;



        //value = heatMap.get(pointY, pointX)[0];
        Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x + templateMat.cols(), matchLoc.y + templateMat.rows()), new Scalar(0, 255, 0));

        //return input;
        return input;
    }
    public int getXPos()
    {
        return xPos;
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