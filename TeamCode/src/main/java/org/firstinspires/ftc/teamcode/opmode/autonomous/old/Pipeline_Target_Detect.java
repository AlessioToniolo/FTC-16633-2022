
package org.firstinspires.ftc.teamcode;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.CvType;
import org.opencv.core.Core;
import org.opencv.core.Core.MinMaxLocResult;


public class Pipeline_Target_Detect extends OpenCvPipeline
{
    public Mat templateMat = new Mat(64, 64, CvType.CV_8UC1);

    public double xPos = -1;
    public double value = -1;
   
   
   
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
        return input; 
    }
    private void buildTemplate()
    {
        int[][] template = new int[64][64];
        for(int r = 0; r < 32; r++)
        {
            for(int c = 0; c < 32; c++)
            {
                template[r][c] = 255;
            }
        }
        for(int r = 32; r < 64; r++)
        {
            for(int c =32; c < 64; c++)
            {
                template[r][c] = 255;
            }
        }
       
       
        for(int r = 0; r < 64; r++)
        {
            for(int c = 0; c < 64; c++)
            {
                templateMat.put(r, c, template[r][c]);
            }
        }
    }
    public double getXPos()
    {
        return xPos;
    }
    public double getValue()
    {
        return value; 
    }

    // todo: write your code here
}
