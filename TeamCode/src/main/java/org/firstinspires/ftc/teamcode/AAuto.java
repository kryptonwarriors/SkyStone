/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import java.util.ArrayList;
import java.util.List;


import static org.firstinspires.ftc.teamcode.opencvSkystoneDetector.StageSwitchingPipeline.Stage.detection;

@Autonomous(name = "AAuto", group = "Iterative Opmode")
public class AAuto extends OpMode {
    private static ElapsedTime runtime = new ElapsedTime();
    private Util autoUility;

    private final int EXIT_TIME_THRESHOLD = 20;
    private static DcMotor LeftForward = null;
    private static DcMotor LeftBack = null;
    private static DcMotor RightForward = null;
    private static DcMotor RightBack = null;
    private static DcMotor LinearActuator = null;
    private static DcMotor LeftCascade = null;
    private static DcMotor RightCascade = null;

    private static Servo LeftClamp = null;
    private static Servo LeftFoundation = null;
    private static Servo RightClamp = null;
    private static Servo RightFoundation = null;

    private static DistanceSensor LeftDistance = null;
    private static DistanceSensor RightDistance = null;
    private static DistanceSensor BackDistance = null;

    private BNO055IMU imu_1;
    private BNO055IMU imu;

    PIDController pidDrive;
    double globalAngle, correction;
    Orientation lastAngles = new Orientation();

    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;

    private static String SkyStonePos;

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;

    private static float offsetX = 0f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f / 8f + offsetX, 4f / 9f + offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f / 11.4f + offsetX, 4f / 9f + offsetY};
    private static float[] rightPos = {5f / 6f + offsetX, 4f / 9f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    private opencvSkystoneDetector.StageSwitchingPipeline.Stage stageToRenderToViewport = detection;
    private opencvSkystoneDetector.StageSwitchingPipeline.Stage[] stages = opencvSkystoneDetector.StageSwitchingPipeline.Stage.values();

    OpenCvCamera webcam = null;

    Mat yCbCrChan2Mat = new Mat();
    Mat thresholdMat = new Mat();
    Mat all = new Mat();
    Mat input = new Mat();
    List<MatOfPoint> contoursList = new ArrayList<>();

    private ColorSensor Color;
    private Blinker Control_Hub;
    private Blinker Expansion_Hub;
    private TouchSensor LFBumper;
    private TouchSensor RFBumper;
    private TouchSensor LBBumper;
    private TouchSensor RBBumper;

    int FORWARD = 0;
    int BACKWARD = 1;
    int LEFT = 2;
    int RIGHT = 3;
    int UP = 4;
    int WALL = 5;
    int RTurn = 6;
    int LTurn = 7;
    int EXTEND = 8;
    int RETRACT = 9;
    int THRESH = 15;
    int ALL_THRESH = 15;
    int TURNTHRESH = 30;
    double OPTIMUM_POWER = 0.4;
    double STRAFE_POWER = 0.5;

    public static ElapsedTime stateTimer = new ElapsedTime();

    //All states will go in here
    public enum RobotState {
        INIT, POSITION_TO_SKYSTONE, GO_TO_SKYSTONE, WAIT, PICK_UP_STONE, GO_TO_FOUNDATION, GO_to_SKYSTONE2, ARM_DOWN, RELEASE, ARM_UP, FOUNDATION_ARM_DOWN,
        GO_BACK_FOR_STONE, END
    }

    RobotState CurrentState = RobotState.INIT;
    /*
     * Code to run ONCE when the driver hits INIT
     */

    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private opencvSkystoneDetector.StageSwitchingPipeline.Stage stageToRenderToViewport = opencvSkystoneDetector.StageSwitchingPipeline.Stage.detection;
        private opencvSkystoneDetector.StageSwitchingPipeline.Stage[] stages = opencvSkystoneDetector.StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int) (input.rows() * midPos[1]), (int) (input.cols() * midPos[0]));//gets value at circle
            valMid = (int) pixMid[0];

            double[] pixLeft = thresholdMat.get((int) (input.rows() * leftPos[1]), (int) (input.cols() * leftPos[0]));//gets value at circle
            valLeft = (int) pixLeft[0];

            double[] pixRight = thresholdMat.get((int) (input.rows() * rightPos[1]), (int) (input.cols() * rightPos[0]));//gets value at circle
            valRight = (int) pixRight[0];

            //create three points
            Point pointMid = new Point((int) (input.cols() * midPos[0]), (int) (input.rows() * midPos[1]));
            Point pointLeft = new Point((int) (input.cols() * leftPos[0]), (int) (input.rows() * leftPos[1]));
            Point pointRight = new Point((int) (input.cols() * rightPos[0]), (int) (input.rows() * rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointMid, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointLeft, 5, new Scalar(255, 0, 0), 1);//draws circle
            Imgproc.circle(all, pointRight, 5, new Scalar(255, 0, 0), 1);//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols() * (leftPos[0] - rectWidth / 1.5),
                            input.rows() * (leftPos[1] - rectHeight / .7)),
                    new Point(
                            input.cols() * (leftPos[0] + rectWidth / 1.5),
                            input.rows() * (leftPos[1] + rectHeight / .7)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols() * (midPos[0] - rectWidth / 1.5),
                            input.rows() * (midPos[1] - rectHeight / .7)),
                    new Point(
                            input.cols() * (midPos[0] + rectWidth / 1.5),
                            input.rows() * (midPos[1] + rectHeight / .7)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols() * (rightPos[0] - rectWidth / 1.5),
                            input.rows() * (rightPos[1] - rectHeight / .7)),
                    new Point(
                            input.cols() * (rightPos[0] + rectWidth / 1.5),
                            input.rows() * (rightPos[1] + rectHeight / .7)),
                    new Scalar(0, 255, 0), 3);

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

    }

    @Override
    public void init() {

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParameters);

        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        RightForward = hardwareMap.dcMotor.get("RightForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        LinearActuator = hardwareMap.dcMotor.get("LinearActuator");
        LeftCascade = hardwareMap.dcMotor.get("LeftCascade");
        RightCascade = hardwareMap.dcMotor.get("RightCascade");

        //BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
        LFBumper = hardwareMap.get(RevTouchSensor.class, "LFBumper");
        RFBumper = hardwareMap.get(RevTouchSensor.class, "RFBumper");
        LBBumper = hardwareMap.get(RevTouchSensor.class, "LBBumper");
        RBBumper = hardwareMap.get(RevTouchSensor.class, "RBBumper");

        LeftFoundation = hardwareMap.servo.get("LeftFoundation");
        RightFoundation = hardwareMap.servo.get("RightFoundation");
        LeftClamp = hardwareMap.servo.get("LeftClamp");
        RightClamp = hardwareMap.servo.get("RightClamp");

        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");

        autoUility = new Util ( runtime, SkyStonePos, hardwareMap );

        //Reset Foundation Servo Positions
        LeftFoundation.setPosition(0.80);
        RightFoundation.setPosition(0.22);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        if (webcam == null) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this
            webcam.openCameraDevice();//open camera
            telemetry.addData("Camera Opened", "");
            telemetry.update();
            webcam.setPipeline(new StageSwitchingPipeline());//different stages
            webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        } else {
            if (valLeft == 0) {
                SkyStonePos = "Left";
            } else if (valMid == 0) {
                SkyStonePos = "Center";
            } else if (valRight == 0) {
                SkyStonePos = "Right";

            }
        }

        if (!imu.isGyroCalibrated()) {
            telemetry.addData(">", "calibrating");
        } else {
            telemetry.addData(">", "calibration done");
        }
        telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
        telemetry.addData("SkyStonePos", SkyStonePos);
        telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
        telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }


    private void StartMotors(int Direction, double Power) {
        LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        THRESH = ALL_THRESH;
        if (Direction == FORWARD) {
            RightForward.setPower(Power);
            LeftBack.setPower(-Power);
            LeftForward.setPower(-Power);
            RightBack.setPower(Power);
        } else if (Direction == BACKWARD) {
            RightForward.setPower(-Power);
            LeftBack.setPower(Power);
            LeftForward.setPower(Power);
            RightBack.setPower(-Power);
        } else if (Direction == LEFT) {
            LeftForward.setPower(Power);
            LeftBack.setPower(-Power);
            RightForward.setPower(Power);
            RightBack.setPower(-Power);
        } else if (Direction == RIGHT) {
            LeftForward.setPower(-Power);
            LeftBack.setPower(Power);
            RightForward.setPower(-Power);
            RightBack.setPower(Power);
        } else if (Direction == RTurn) {
            THRESH = TURNTHRESH;
            LeftForward.setPower(-Power);
            LeftBack.setPower(-Power);
            RightForward.setPower(-Power);
            RightBack.setPower(-Power);
        } else if (Direction == LTurn) {
            THRESH = TURNTHRESH;
            LeftForward.setPower(Power);
            LeftBack.setPower(Power);
            RightForward.setPower(Power);
            RightBack.setPower(Power);
        }
    }

    private void DriveWithPID(int direction, double power) {
        if (direction == RIGHT) {
            correction = pidDrive.performPID(getAngle());
            LeftForward.setPower(-power);
            LeftBack.setPower(power);
            RightForward.setPower(-power);
            RightBack.setPower(power);

        } else if (direction == LEFT) {
            correction = pidDrive.performPID(getAngle());
            LeftForward.setPower(power - correction);
            LeftBack.setPower(-power - correction);
            RightForward.setPower(power - correction);
            RightBack.setPower(-power - correction);

        } else if (direction == BACKWARD) {
            correction = pidDrive.performPID(getAngle());
            RightForward.setPower(-power - correction);
            LeftBack.setPower(power - correction);
            LeftForward.setPower(power - correction);
            RightBack.setPower(-power - correction);

        } else if (direction == FORWARD) {
            correction = pidDrive.performPID(getAngle());
            RightForward.setPower(power - correction);
            LeftBack.setPower(-power - correction);
            LeftForward.setPower(-power - correction);
            RightBack.setPower(power - correction);

        }


    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -90)
            deltaAngle += 360;
        else if (deltaAngle > 90)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private void StopDrive() {
        LeftBack.setPower(0.0);
        LeftForward.setPower(0.0);
        RightForward.setPower(0.0);
        RightBack.setPower(0.0);
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();

        pidDrive = new PIDController(-0.8, 0, 0);
        pidDrive.setSetpoint(0);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        Util.Exit ES;
        switch (CurrentState) {

            case INIT:
                stateTimer.reset();
                pidDrive.reset();
                if (SkyStonePos.equals("Left")) {
                    DriveWithPID(LEFT, STRAFE_POWER);
                } else if (SkyStonePos.equals("Right")) {
                    DriveWithPID(RIGHT, STRAFE_POWER);
                } else if (SkyStonePos.equals("Center")) {

                }
                CurrentState = RobotState.POSITION_TO_SKYSTONE;
                break;

            case POSITION_TO_SKYSTONE:
                pidDrive.reset();
                ES = CanIExitPositionToSkyStone();
                if (ES == Util.Exit.ExitState) {
                    StopDrive();
                    DriveWithPID(FORWARD, OPTIMUM_POWER);
                    CurrentState = RobotState.GO_TO_SKYSTONE;
                } else if (ES == Util.Exit.NoTimeLeftExit) {
                    telemetry.addData("Exiting Out with No Time", "");
                    CurrentState = RobotState.GO_TO_FOUNDATION;
                } else if (ES == Util.Exit.DontExit) {

                }
                telemetry.addData("Time Left", 30 - runtime.seconds());
                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("CurrentState", "POSITION_TO_SKYSTONE");
                telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

                break;

            case GO_TO_SKYSTONE:
                pidDrive.reset();
                ES = autoUility.CanIExitGoToSkyStone();
                if (ES == Util.Exit.ExitState) {
                    StopDrive();
                    stateTimer.reset();
                    CurrentState = RobotState.PICK_UP_STONE;
                } else if (ES == Util.Exit.NoTimeLeftExit) {
                    telemetry.addData("Exiting Out with No Time", "");
                    DriveWithPID(RIGHT, OPTIMUM_POWER);
                    CurrentState = RobotState.GO_TO_FOUNDATION;
                } else if (ES == Util.Exit.DontExit) {

                }
                telemetry.addData("Time Left", 30 - runtime.seconds());
                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("CurrentState", "GO_TO_SKYSTONE");
                telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();

                break;

            case PICK_UP_STONE:
                pidDrive.reset();
                ES = autoUility.CanIExitPickUpStone();
                if (ES == Util.Exit.ExitState) {
                    DriveWithPID(RIGHT, OPTIMUM_POWER);
                    CurrentState = RobotState.GO_TO_FOUNDATION;
                } else if (ES == Util.Exit.NoTimeLeftExit) {
                    StopDrive();
                    CurrentState = RobotState.FOUNDATION_ARM_DOWN;
                } else if (ES == Util.Exit.DontExit) {

                }
                telemetry.addData("Time Left", 30 - runtime.seconds());
                telemetry.addData("CurrentState", "PICK_UP_STONE");
                telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
                break;

            case GO_TO_FOUNDATION:
                pidDrive.reset();
                ES = autoUility.CanIExitGoToFoundation();
                if (ES == Util.Exit.ExitState) {
                    StopDrive();
                    stateTimer.reset();
                    CurrentState = RobotState.ARM_DOWN;
                } else if (ES == Util.Exit.NoTimeLeftExit) {
                    StopDrive();
                    CurrentState = RobotState.FOUNDATION_ARM_DOWN;
                } else if (ES == Util.Exit.DontExit) {
                    DriveWithPID(RIGHT, OPTIMUM_POWER);
                }
                telemetry.addData("Time Left", 30 - runtime.seconds());
                telemetry.addData("CurrentState", "GO_TO_FOUNDATION");
                telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
                break;

            case ARM_DOWN:
                pidDrive.reset();
                ES = autoUility.CanIExitArmDown();
                if (ES == Util.Exit.ExitState) {
                    StopDrive();
                    DriveWithPID(LEFT, STRAFE_POWER);
                    CurrentState = RobotState.GO_BACK_FOR_STONE;
                } else if (ES == Util.Exit.NoTimeLeftExit) {
                    StopDrive();
                    DriveWithPID(LEFT, STRAFE_POWER);
                    CurrentState = RobotState.GO_TO_FOUNDATION;
                } else if (ES == Util.Exit.DontExit) {
                    telemetry.addData("CurrentState", "ARM_DOWN");
                    telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }

                break;
            case GO_BACK_FOR_STONE:
               ES = CanIExitGoBackToStone();
                if (ES == Util.Exit.ExitState) {
                    StopDrive();
                    CurrentState = RobotState.GO_TO_FOUNDATION;
                } else if (ES == Util.Exit.NoTimeLeftExit) {
                    StopDrive();
                    CurrentState = RobotState.PICK_UP_STONE;
                } else if (ES == Util.Exit.DontExit) {
                    telemetry.addData("CurrentState", "GO_BACK");
                    telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                    telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }

                break;

            case END:
                StopDrive();
                telemetry.addData("State", "END");
                telemetry.update();
                break;

        }
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    public Util.Exit CanIExitPositionToSkyStone() {

        if (runtime.seconds() > EXIT_TIME_THRESHOLD) {
            return Util.Exit.NoTimeLeftExit;
        }
        // center = 31 || left = 23 || right = 40
        if (SkyStonePos == "Left") {
            if (LeftDistance.getDistance(DistanceUnit.INCH) <= 23) {
                return Util.Exit.ExitState;
            }

        } else if (SkyStonePos == "Right") {
            if (LeftDistance.getDistance(DistanceUnit.INCH) >= 40) {
                return Util.Exit.ExitState;
            }

        } else if (SkyStonePos == "Center") {
            return Util.Exit.ExitState;
        }


        return Util.Exit.DontExit;

    }

    public Util.Exit CanIExitGoBackToStone() {

        if (runtime.seconds() > EXIT_TIME_THRESHOLD) {
            return Util.Exit.NoTimeLeftExit;
        }
        // center = 5 || left = 1 || right = 14
        if (SkyStonePos == "Left") {
            if (LeftDistance.getDistance(DistanceUnit.INCH) <= 1) {
                return Util.Exit.ExitState;
            }

        } else if (SkyStonePos == "Right") {
            if (LeftDistance.getDistance(DistanceUnit.INCH) >= 14) {
                return Util.Exit.ExitState;
            }

        } else if (SkyStonePos == "Center") {
            if (LeftDistance.getDistance(DistanceUnit.INCH) <= 5) {
                return Util.Exit.ExitState;
            }

        }

        return Util.Exit.DontExit;
    }
}
