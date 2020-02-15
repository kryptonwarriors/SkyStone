package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.opencvSkystoneDetector.StageSwitchingPipeline.Stage.detection;


@Autonomous(name = "openCVNORMALOPMODE", group = "6 Stone Auto")
public class openCVNORMALOPMODE extends OpMode {
    private static float rectHeight = .6f / 8f;


    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static String SkyStonePos;
    private static float rectWidth = 1.5f / 8f;
    private static float offsetX = 0f / 8f;
    //changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;
    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    //changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
    Mat yCbCrChan2Mat = new Mat ();
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    private opencvSkystoneDetector.StageSwitchingPipeline.Stage stageToRenderToViewport = detection;
    Mat thresholdMat = new Mat ();

    OpenCvCamera webcam = null;
    Mat all = new Mat ();
    Mat input = new Mat ();
    List<MatOfPoint> contoursList = new ArrayList<> ();
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime ();
    private opencvSkystoneDetector.StageSwitchingPipeline.Stage[] stages =
            opencvSkystoneDetector.StageSwitchingPipeline.Stage.values ();


    @Override
    public void init() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        if (webcam == null) {
            int cameraMonitorViewId =
                    hardwareMap.appContext.getResources ().getIdentifier ( "cameraMonitorViewId",
                                                                           "id",
                                                                           hardwareMap.appContext.getPackageName () );

            //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
            webcam = OpenCvCameraFactory.getInstance ().createWebcam (
                    hardwareMap.get ( WebcamName.class, "Webcam 1" ),
                    cameraMonitorViewId );        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

            webcam.openCameraDevice ();//open camera
            telemetry.addData ( "Camera Opened", "" );
            telemetry.update ();
            webcam.setPipeline ( new StageSwitchingPipeline () );//different stages
            webcam.startStreaming ( rows, cols, OpenCvCameraRotation.UPRIGHT );//display on RC
        }


        if (valLeft == 0) {
            SkyStonePos = "Left";
        } else if (valMid == 0) {
            SkyStonePos = "Center";
        } else if (valRight == 0) {
            SkyStonePos = "Right";
        }
        telemetry.addData ( "Values", valLeft + "   " + valMid + "   " + valRight );
        telemetry.addData ( "SkyStonePos", SkyStonePos );
        telemetry.addData ( "Height", rows );
        telemetry.addData ( "Width", cols );

        telemetry.update ();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset ();
    }

    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat ();
        Mat thresholdMat = new Mat ();
        Mat all = new Mat ();
        List<MatOfPoint> contoursList = new ArrayList<> ();
        private opencvSkystoneDetector.StageSwitchingPipeline.Stage stageToRenderToViewport =
                opencvSkystoneDetector.StageSwitchingPipeline.Stage.detection;
        private opencvSkystoneDetector.StageSwitchingPipeline.Stage[] stages =
                opencvSkystoneDetector.StageSwitchingPipeline.Stage.values ();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal ();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear ();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor ( input, yCbCrChan2Mat,
                               Imgproc.COLOR_RGB2YCrCb );//converts rgb to ycrcb
            Core.extractChannel ( yCbCrChan2Mat, yCbCrChan2Mat, 2 );//takes cb difference and stores

            //b&w
            Imgproc.threshold ( yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV );

            //outline/contour
            Imgproc.findContours ( thresholdMat, contoursList, new Mat (), Imgproc.RETR_LIST,
                                   Imgproc.CHAIN_APPROX_SIMPLE );
            yCbCrChan2Mat.copyTo ( all );//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get ( (int) (input.rows () * midPos[1]),
                                                 (int) (input.cols () * midPos[0]) );//gets value at circle
            valMid = (int) pixMid[0];

            double[] pixLeft = thresholdMat.get ( (int) (input.rows () * leftPos[1]),
                                                  (int) (input.cols () * leftPos[0]) );//gets value at circle
            valLeft = (int) pixLeft[0];

            double[] pixRight = thresholdMat.get ( (int) (input.rows () * rightPos[1]),
                                                   (int) (input.cols () * rightPos[0]) );//gets value at circle
            valRight = (int) pixRight[0];

            //create three points
            Point pointMid = new Point ( (int) (input.cols () * midPos[0]),
                                         (int) (input.rows () * midPos[1]) );
            Point pointLeft = new Point ( (int) (input.cols () * leftPos[0]),
                                          (int) (input.rows () * leftPos[1]) );
            Point pointRight = new Point ( (int) (input.cols () * rightPos[0]),
                                           (int) (input.rows () * rightPos[1]) );

            //draw circles on those points
            Imgproc.circle ( all, pointMid, 5, new Scalar ( 255, 0, 0 ), 1 );//draws circle
            Imgproc.circle ( all, pointLeft, 5, new Scalar ( 255, 0, 0 ), 1 );//draws circle
            Imgproc.circle ( all, pointRight, 5, new Scalar ( 255, 0, 0 ), 1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle (//1-3
                               all,
                               new Point (
                                       input.cols () * (leftPos[0] - rectWidth),
                                       input.rows () * (leftPos[1] - rectHeight) ),
                               new Point (
                                       input.cols () * (leftPos[0] + rectWidth),
                                       input.rows () * (leftPos[1] + rectHeight) ),
                               new Scalar ( 0, 155, 0 ), 3 );
            Imgproc.rectangle (//3-5
                               all,
                               new Point (
                                       input.cols () * (midPos[0] - rectWidth / 2),
                                       input.rows () * (midPos[1] - rectHeight / 2) ),
                               new Point (
                                       input.cols () * (midPos[0] + rectWidth / 2),
                                       input.rows () * (midPos[1] + rectHeight / 2) ),
                               new Scalar ( 0, 255, 0 ), 3 );
            Imgproc.rectangle (//5-7
                               all,
                               new Point (
                                       input.cols () * (rightPos[0] - rectWidth / 2),
                                       input.rows () * (rightPos[1] - rectHeight / 2) ),
                               new Point (
                                       input.cols () * (rightPos[0] + rectWidth / 2),
                                       input.rows () * (rightPos[1] + rectHeight / 2) ),
                               new Scalar ( 0, 255, 0 ), 3 );

            switch (stageToRenderToViewport) {
                case THRESHOLD: {
                    return thresholdMat;
                }

                case detection: {
                    return all;
                }

                case RAW_IMAGE: {
                    return input;
                }

                default: {
                    return input;
                }
            }
        }

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
