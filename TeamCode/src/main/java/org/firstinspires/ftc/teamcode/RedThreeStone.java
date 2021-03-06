package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
import java.util.SplittableRandom;

/**
 * Created by maryjaneb  on 11/13/2016.
 * <p>
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 * <p>
 * monitor: 640 x 480
 * YES
 */
@Autonomous(name = "RedThreeStone", group = "Sky autonomous")

//@Disabled//comment out this line before using
public class RedThreeStone extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255

    private static DcMotor LeftForward = null;
    private static DcMotor LeftBack = null;
    private static DcMotor RightForward = null;
    private static DcMotor RightBack = null;
    private static DcMotor LinearActuator = null;
    private static DcMotor LeftCascade = null;
    private static DcMotor RightCascade = null;

    private static Servo LeftClamp = null;
    private static Servo RightClamp = null;
    private static Servo RightTurner = null;
    private static Servo RightClamper = null;
    private static Servo LeftTurner = null;
    private static Servo LeftClamper = null;


    private static DistanceSensor LeftDistance = null;
    private static DistanceSensor RightDistance = null;
    private static DistanceSensor BackDistance = null;


    private BNO055IMU imu;

    PIDController pidDrive;
    PIDController RightpidDrive;
    PIDController ForwardpidDrive;
    PIDController LeftpidDrive;
    PIDController testPIDDrive;
    double globalAngle, correction;
    Orientation lastAngles = new Orientation();

    private ColorSensor Color;
    private Blinker Control_Hub;
    private Blinker Expansion_Hub;
    private TouchSensor LFBumper;
    private TouchSensor RFBumper;
    private TouchSensor LBBumper;
    private TouchSensor RBBumper;
    private Orientation angles;

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
    double STRAFE_POWER = 0.75;

    private enum autoServoStates {
        INIT, GRAB, DROP, AWAY
    }

    public static ElapsedTime timer = new ElapsedTime();


    private static int valMid = -1;
    private static int valLeft = -1;
    private static int valRight = -1;
    private static String SkyStonePos;

    private static float rectHeight = .6f / 8f;
    private static float rectWidth = 1.5f / 8f;

    private static float offsetX = -0.77f / 7f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 0f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;


    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //P.S. if you're using the latest version of easyopencv, you might need to change the next line to the following:
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);//remove this

        webcam.openCameraDevice();//open camera
        webcam.setPipeline(new StageSwitchingPipeline());//different stages
        webcam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

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


        LeftClamp = hardwareMap.servo.get("LeftClamp");
        RightClamp = hardwareMap.servo.get("RightClamp");
        RightTurner = hardwareMap.servo.get("RightTurner");
        RightClamper = hardwareMap.servo.get("RightClamper");
        LeftTurner = hardwareMap.servo.get("LeftTurner");
        LeftClamper = hardwareMap.servo.get("LeftClamper");


        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");

        RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        RightForward.setDirection(DcMotorSimple.Direction.REVERSE);


        RightpidDrive = new PIDController(0.05, 0, 0);
        RightpidDrive.setSetpoint(0);
        RightpidDrive.setOutputRange(0, STRAFE_POWER);
        RightpidDrive.setInputRange(-90, 90);
        RightpidDrive.enable();


        turnServo(autoServoStates.INIT, RIGHT);
        turnServo(autoServoStates.INIT, LEFT);
        clampServo(autoServoStates.INIT, LEFT);
        clampServo(autoServoStates.INIT, RIGHT);


        while (!(isStopRequested() || isStarted())) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            correction = RightpidDrive.performPID(getAngle());


            if (valLeft == 0) {
                SkyStonePos = "Left";
            } else if (valMid == 0) {
                SkyStonePos = "Center";
            } else if (valRight == 0) {
                SkyStonePos = "Right";
            }
            telemetry.addData("correction", correction);
            telemetry.addData("rot about Z", angles.firstAngle);
            telemetry.addData("rot about Y", angles.secondAngle);
            telemetry.addData("rot about X", angles.thirdAngle);
            telemetry.addData("LeftBackBumper", LBBumper.isPressed());
            telemetry.addData("RightBackBumper", RBBumper.isPressed());
            telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Values", valLeft + "   " + valMid + "   " + valRight);
            telemetry.addData("SkyStonePos", SkyStonePos);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();

        }

        runtime.reset();


        if (opModeIsActive()) {

            //Arm Down
            //BackTurner.setPosition(0.6);
            //BackClamper.setPosition(0.1);

            ForwardpidDrive = new PIDController(0.01, 0, 0);
            ForwardpidDrive.setSetpoint(0);
            ForwardpidDrive.setOutputRange(0, STRAFE_POWER);
            ForwardpidDrive.setInputRange(-90, 90);
            ForwardpidDrive.enable();
            pidDrive = new PIDController(0.012, 0.0001, 0);
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, STRAFE_POWER);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();



/*
            StartMotors(LTurn, 0.5);
            while (getAngle() > -72 && !(isStopRequested())) {
                telemetry.addData("angle", getAngle());
                telemetry.update();
                //if (getAngle() < -80 && () > -100)
                    //break;
            }
            StopDrive();

*/


            if (SkyStonePos.equals("Left")) {
                DriveWithLeftDistance(LEFT, STRAFE_POWER, 34);

                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, RIGHT);


                DriveWithBackDistance(FORWARD, 0.3, 25, 300);


                clampServo(autoServoStates.GRAB, RIGHT);
                sleep(600);
                turnServo(autoServoStates.AWAY, RIGHT);
                sleep(200);

                DriveWithBackDistance(BACKWARD, 0.3, 25, 50);
                sleep(50);
                DriveWithPID(RIGHT, STRAFE_POWER, 700);
                DriveWithRightDistance(RIGHT, STRAFE_POWER, 26);
                sleep(50);
                moveUntilBackBumper(0.3);
                turnServo(autoServoStates.GRAB, RIGHT);
                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, LEFT);
                sleep(600);

                DriveWithBackDistance(BACKWARD, 0.4, 1.5, 100);

                // DriveWithRightDistance(LEFT, STRAFE_POWER, 28);

                StartMotors(LTurn, 0.4);
                sleep(700);
                StopDrive();

                turnServo(autoServoStates.AWAY, LEFT);
                turnServo(autoServoStates.AWAY, RIGHT);
                clampServo(autoServoStates.INIT, LEFT);
                clampServo(autoServoStates.INIT, RIGHT);


                DriveWithPID(LEFT, STRAFE_POWER, 1200);
                DriveWithLeftDistance(LEFT, STRAFE_POWER, 10);
                StopDrive();


                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, RIGHT);

                DriveWithBackDistance(FORWARD, 0.3, 25.5, 100);

                clampServo(autoServoStates.GRAB, RIGHT);
                sleep(600);
                turnServo(autoServoStates.AWAY, RIGHT);
                sleep(200);


                DriveWithBackDistance(BACKWARD, 0.3, 4, 100);
                sleep(50);
                DriveWithPID(RIGHT, STRAFE_POWER, 700);
                DriveWithRightDistance(RIGHT, STRAFE_POWER, 28);

                turnServo(autoServoStates.GRAB, RIGHT);
                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, LEFT);
                sleep(400);


                turnServo(autoServoStates.AWAY, LEFT);
                turnServo(autoServoStates.AWAY, RIGHT);
                clampServo(autoServoStates.INIT, LEFT);
                clampServo(autoServoStates.INIT, RIGHT);

                DriveWithPID(LEFT, STRAFE_POWER, 950);
                StopDrive();
            } else if (SkyStonePos.equals("Right")) {

                clampServo(autoServoStates.DROP, LEFT);
                turnServo(autoServoStates.GRAB, LEFT);


                DriveWithBackDistance(FORWARD, 0.3, 25, 300);


                clampServo(autoServoStates.GRAB, LEFT);
                sleep(600);
                turnServo(autoServoStates.AWAY, LEFT);
                sleep(200);


                DriveWithPID(BACKWARD, 0.3, 30);
                StopDrive();
                DriveWithPID(RIGHT, STRAFE_POWER, 700);
                DriveWithRightDistance(RIGHT, STRAFE_POWER, 26);
                sleep(50);
                moveUntilBackBumper(0.25);
                turnServo(autoServoStates.GRAB, LEFT);
                clampServo(autoServoStates.DROP, LEFT);
                turnServo(autoServoStates.GRAB, RIGHT);
                sleep(600);

                turnServo(autoServoStates.INIT, LEFT);
                turnServo(autoServoStates.INIT, RIGHT);
                clampServo(autoServoStates.INIT, LEFT);
                clampServo(autoServoStates.INIT, RIGHT);

                DriveWithPID(BACKWARD, 0.3, 60);
                StopDrive();


                DriveWithPID(LEFT, STRAFE_POWER, 1200);
                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, RIGHT);
                DriveWithLeftDistance(LEFT, STRAFE_POWER, 32);


                DriveWithBackDistance(FORWARD, 0.3, 25.5, 60);

                clampServo(autoServoStates.GRAB, RIGHT);
                sleep(500);
                turnServo(autoServoStates.AWAY, RIGHT);
                sleep(200);


                DriveWithPID(BACKWARD, 0.3, 30);
                StopDrive();
                DriveWithPID(RIGHT, STRAFE_POWER, 700);
                DriveWithRightDistance(RIGHT, STRAFE_POWER, 28);

                moveUntilBackBumper(0.25);

                turnServo(autoServoStates.GRAB, RIGHT);
                sleep(180);
                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, LEFT);
                sleep(400);


                turnServo(autoServoStates.INIT, LEFT);
                turnServo(autoServoStates.INIT, RIGHT);
                clampServo(autoServoStates.INIT, LEFT);
                clampServo(autoServoStates.INIT, RIGHT);
                DriveWithPID(BACKWARD, 0.3, 55);
                StopDrive();

                DriveWithPID(LEFT, STRAFE_POWER, 1200);
                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, RIGHT);
                DriveWithLeftDistance(LEFT, STRAFE_POWER, 40);

                DriveWithBackDistance(FORWARD, 0.3, 27, 50);

                clampServo(autoServoStates.GRAB, RIGHT);
                sleep(520);
                turnServo(autoServoStates.AWAY, RIGHT);
                sleep(200);

                DriveWithPID(BACKWARD, 0.3, 40);
                StopDrive();
                DriveWithPID(RIGHT, STRAFE_POWER, 700);
                DriveWithRightDistance(RIGHT, STRAFE_POWER, 28);

                moveUntilBackBumper(0.25);

                turnServo(autoServoStates.GRAB, RIGHT);
                sleep(180);
                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, LEFT);
                sleep(400);


                turnServo(autoServoStates.AWAY, LEFT);
                turnServo(autoServoStates.AWAY, RIGHT);
                clampServo(autoServoStates.INIT, LEFT);
                clampServo(autoServoStates.INIT, RIGHT);
                DriveWithBackDistance(BACKWARD, 0.3, 24, 60);
                StopDrive();
                DriveWithPID(LEFT, STRAFE_POWER, 850);
                StopDrive();
            } else if (SkyStonePos.equals("Center")) {

                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, RIGHT);


                DriveWithBackDistance(FORWARD, 0.3, 25, 100);


                clampServo(autoServoStates.GRAB, RIGHT);
                sleep(600);
                turnServo(autoServoStates.AWAY, RIGHT);
                sleep(200);

                DriveWithBackDistance(BACKWARD, 0.3, 20, 100);
                sleep(50);
                DriveWithPID(RIGHT, STRAFE_POWER, 700);
                DriveWithRightDistance(RIGHT, STRAFE_POWER, 26);
                sleep(50);
                moveUntilBackBumper(0.3);
                turnServo(autoServoStates.GRAB, RIGHT);
                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, LEFT);
                sleep(600);

                DriveWithBackDistance(BACKWARD, 0.4, 24, 100);

                // DriveWithRightDistance(LEFT, STRAFE_POWER, 28);

                StartMotors(LTurn, 0.4);
                sleep(700);
                StopDrive();

                turnServo(autoServoStates.AWAY, LEFT);
                turnServo(autoServoStates.AWAY, RIGHT);
                clampServo(autoServoStates.INIT, LEFT);
                clampServo(autoServoStates.INIT, RIGHT);


                DriveWithPID(LEFT, STRAFE_POWER, 1200);
                DriveWithLeftDistance(LEFT, STRAFE_POWER, 24);
                StopDrive();


                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, RIGHT);

                DriveWithBackDistance(FORWARD, 0.3, 25.5, 100);

                clampServo(autoServoStates.GRAB, RIGHT);
                sleep(600);
                turnServo(autoServoStates.AWAY, RIGHT);
                sleep(200);


                DriveWithBackDistance(BACKWARD, 0.3, 20, 100);
                sleep(50);
                DriveWithPID(RIGHT, STRAFE_POWER, 700);
                DriveWithRightDistance(RIGHT, STRAFE_POWER, 28);

                turnServo(autoServoStates.GRAB, RIGHT);
                clampServo(autoServoStates.DROP, RIGHT);
                turnServo(autoServoStates.GRAB, LEFT);
                sleep(400);


                turnServo(autoServoStates.AWAY, LEFT);
                turnServo(autoServoStates.AWAY, RIGHT);
                clampServo(autoServoStates.INIT, LEFT);
                clampServo(autoServoStates.INIT, RIGHT);

                DriveWithPID(LEFT, STRAFE_POWER, 950);


            }


//END


















            /*
            //clamp
            BackClamper.setPosition(1);
            sleep(1000);

            //arm up
            BackTurner.setPosition(0);
            sleep(500);
            DriveWithPID(FORWARD, 0.3, 10);
            //DriveWithLeftDistance(LEFT, 0.3, 18);
            //StartMotors(BACKWARD, 0.4);
            RightpidDrive.reset();
            DriveLeftPid(200, 0.4);
            sleep(3000);
            DriveWithPID(BACKWARD, 0.4, 300);
            while(!(RBBumper.isPressed() && LBBumper.isPressed())) {
                DriveWithPID(BACKWARD, 0.2, 100);
                telemetry.addData("not touched", "not pressed");
                telemetry.update();
            }
            StopDrive();
/*
            //Arm Down
            BackTurner.setPosition(0.55);
            //unclamp
            BackClamper.setPosition(0.3);
            //arm up
            BackTurner.setPosition(0);*/

            /*DriveWithPID(LEFT, 0.6, 1500);
            if(SkyStonePos == "Left")
                DriveWithLeftDistance(LEFT, 0.4, 2);
            if(SkyStonePos == "Center")
                DriveWithLeftDistance(LEFT, 0.4, 10);
            if(SkyStonePos == "Right")
                DriveWithLeftDistance(LEFT, 0.4, 22);

            DriveWithBackDistance(BACKWARD, 0.3, 18);

            //Arm Down
            //clamp
            //arm up

            DriveWithPID(RIGHT, 0.6, 1800);
*/


        }

        while (opModeIsActive()) {
            correction = RightpidDrive.performPID(getAngle());
            telemetry.addData("correction", correction);
            telemetry.addData("getAngle", getAngle());
            telemetry.update();

        }


    }


    //FUNCTIONS
    private void moveUntilBackBumper(double Power) {
        correction = ForwardpidDrive.performPID(getAngle());
        RightForward.setPower(Power - correction);
        LeftBack.setPower(Power + correction);
        LeftForward.setPower(Power + correction);
        RightBack.setPower(Power - correction);
        while (!(LBBumper.isPressed() || RBBumper.isPressed())) {
            telemetry.addData("LeftBackBumper", LBBumper.isPressed());
            telemetry.addData("RightBackBumper", RBBumper.isPressed());
            telemetry.addData("correction", correction);
            telemetry.update();
        }
        StopDrive();
    }

    private void moveUntilFrontBumper(double Power) {
        correction = pidDrive.performPID(getAngle());
        RightForward.setPower(-Power - correction);
        LeftBack.setPower(-Power + correction);
        LeftForward.setPower(-Power + correction);
        RightBack.setPower(-Power - correction);
        while (!(LFBumper.isPressed() || RFBumper.isPressed())) {
            telemetry.addData("LeftFrontBumper", LFBumper.isPressed());
            telemetry.addData("RightFrontBumper", RFBumper.isPressed());
            telemetry.addData("correction", correction);
            telemetry.update();
        }
        StopDrive();
    }

    private void StartMotors(int Direction, double Power) {
        correction = pidDrive.performPID(getAngle());
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
            RightForward.setPower(Power - correction);
            LeftBack.setPower(Power + correction);
            LeftForward.setPower(Power + correction);
            RightBack.setPower(Power - correction);
        } else if (Direction == BACKWARD) {
            RightForward.setPower(-Power);
            LeftBack.setPower(-Power);
            LeftForward.setPower(-Power);
            RightBack.setPower(-Power);
        } else if (Direction == LEFT) {
            LeftForward.setPower(Power);
            LeftBack.setPower(-Power);
            RightForward.setPower(Power);
            RightBack.setPower(-Power);
        } else if (Direction == RIGHT) {
            LeftForward.setPower(Power);
            LeftBack.setPower(-Power);
            RightForward.setPower(-Power);
            RightBack.setPower(Power);
        } else if (Direction == RTurn) {
            THRESH = TURNTHRESH;
            LeftForward.setPower(Power);
            LeftBack.setPower(Power);
            RightForward.setPower(-Power);
            RightBack.setPower(-Power);
        } else if (Direction == LTurn) {
            THRESH = TURNTHRESH;
            LeftForward.setPower(-Power);
            LeftBack.setPower(-Power);
            RightForward.setPower(Power);
            RightBack.setPower(Power);
        }
    }

    private void DriveWithPID(int direction, double power, int TargetPosition) {
        LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double LFpower, RFpower, LBpower, RBpower;

        if (direction == BACKWARD) {
            LeftForward.setTargetPosition(-TargetPosition);
            RightForward.setTargetPosition(TargetPosition);
            LeftBack.setTargetPosition(-TargetPosition);
            RightBack.setTargetPosition(TargetPosition);
            LFpower = -power;
            RFpower = -power;
            LBpower = -power;
            RBpower = -power;
        } else if (direction == FORWARD) {
            LeftForward.setTargetPosition(TargetPosition);
            RightForward.setTargetPosition(-TargetPosition);
            LeftBack.setTargetPosition(TargetPosition);
            RightBack.setTargetPosition(-TargetPosition);
            LFpower = power;
            RFpower = power;
            LBpower = power;
            RBpower = power;
        } else if (direction == RIGHT) {
            LeftForward.setTargetPosition(TargetPosition);
            RightForward.setTargetPosition(TargetPosition);
            LeftBack.setTargetPosition(-TargetPosition);
            RightBack.setTargetPosition(-TargetPosition);
            LFpower = power;
            RFpower = -power;
            LBpower = -power;
            RBpower = power;
        } else if (direction == LEFT) {
            LeftForward.setTargetPosition(-TargetPosition);
            RightForward.setTargetPosition(-TargetPosition);
            LeftBack.setTargetPosition(TargetPosition);
            RightBack.setTargetPosition(TargetPosition);
            LFpower = -power;
            RFpower = power;
            LBpower = power;
            RBpower = -power;
        } else {
            LFpower = 0;
            RFpower = 0;
            LBpower = 0;
            RBpower = 0;
        }

        //Math.abs(LeftForward.getCurrentPosition()) < Math.abs(LeftForward.getTargetPosition())
        while (opModeIsActive() && Math.abs(LeftForward.getCurrentPosition()) < Math.abs(LeftForward.getTargetPosition())) {

            if (direction == RIGHT)
                correction = pidDrive.performPID(getAngle());
            if (direction == LEFT)
                correction = pidDrive.performPID(getAngle());
            else
                correction = pidDrive.performPID(getAngle());
            LeftForward.setPower(LFpower + correction);
            LeftBack.setPower(LBpower + correction);
            RightForward.setPower(RFpower - correction);
            RightBack.setPower(RBpower - correction);

            telemetry.addData("correction", correction);
            telemetry.addData("LeftForward", LeftForward.getPower());
            telemetry.addData("RightForward", RightForward.getPower());
            telemetry.addData("LeftBack", LeftBack.getPower());
            telemetry.addData("RightBack", RightBack.getPower());
            telemetry.addData("CurrentPos", LeftForward.getCurrentPosition());
            telemetry.addData("TargetPos", LeftForward.getTargetPosition());
            telemetry.addData("reached TargetPos?", Math.abs(LeftForward.getCurrentPosition()) < Math.abs(LeftForward.getTargetPosition()));
            telemetry.update();
        }
        sleep(100);
    }

    private void DriveWithBackDistance(int direction, double power, double inches, int encoders) {
        if (direction == FORWARD) {
            LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftForward.setTargetPosition(encoders);

            while (opModeIsActive() && (BackDistance.getDistance(DistanceUnit.INCH) < inches || LeftForward.getTargetPosition() > encoders)) {
                correction = pidDrive.performPID(getAngle());

                LeftForward.setPower(power + correction);
                LeftBack.setPower(power + correction);
                RightForward.setPower(power - correction);
                RightBack.setPower(power - correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } else if (direction == BACKWARD) {
            LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftForward.setTargetPosition(-encoders);
            while (opModeIsActive() && (BackDistance.getDistance(DistanceUnit.INCH) > inches || LeftForward.getTargetPosition() < encoders)) {
                correction = pidDrive.performPID(getAngle());
                LeftForward.setPower(-power + correction);
                LeftBack.setPower(-power + correction);
                RightForward.setPower(-power - correction);
                RightBack.setPower(-power - correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }

        StopDrive();
        sleep(200);
    }

    private void DriveWithRightDistance(int direction, double power, double inches) {
        if (direction == LEFT) {
            while (opModeIsActive() && RightDistance.getDistance(DistanceUnit.INCH) < inches) {
                correction = pidDrive.performPID(getAngle());
                LeftForward.setPower(-power + correction);
                LeftBack.setPower(power + correction);
                RightForward.setPower(power - correction);
                RightBack.setPower(-power - correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } else if (direction == RIGHT) {
            while (opModeIsActive() && RightDistance.getDistance(DistanceUnit.INCH) > inches) {
                correction = pidDrive.performPID(getAngle());
                LeftForward.setPower(power + correction);
                LeftBack.setPower(-power + correction);
                RightForward.setPower(-power - correction);
                RightBack.setPower(power - correction);


                telemetry.addData("globalAngle", globalAngle);
                telemetry.addData("getAngle", getAngle());
                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }

        StopDrive();
        sleep(200);
    }

    private void DriveLeftPid(int TargetPos, double power) {
        while (opModeIsActive() && LeftForward.getCurrentPosition() >= LeftForward.getTargetPosition()) {
            correction = LeftpidDrive.performPID(getAngle());
            LeftForward.setPower(-power - correction);
            LeftBack.setPower(power - correction);
            RightForward.setPower(power - correction);
            RightBack.setPower(-power - correction);

            telemetry.addData("correction", correction);
            telemetry.addData("LeftForward", LeftForward.getPower());
            telemetry.addData("RightForward", RightForward.getPower());
            telemetry.addData("LeftBack", LeftBack.getPower());
            telemetry.addData("RightBack", RightBack.getPower());
            telemetry.addData("RightDistance", LeftDistance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    private void DriveWithLeftDistance(int direction, double power, double inches) {
        if (direction == LEFT) {
            while (opModeIsActive() && LeftDistance.getDistance(DistanceUnit.INCH) > inches) {
                correction = pidDrive.performPID(getAngle());
                LeftForward.setPower(-power + correction);
                LeftBack.setPower(power + correction);
                RightForward.setPower(power - correction);
                RightBack.setPower(-power - correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } else if (direction == RIGHT) {
            while (opModeIsActive() && LeftDistance.getDistance(DistanceUnit.INCH) < inches) {
                correction = pidDrive.performPID(getAngle());
                LeftForward.setPower(power + correction);
                LeftBack.setPower(-power + correction);
                RightForward.setPower(-power - correction);
                RightBack.setPower(power - correction);

                telemetry.addData("correction", correction);
                telemetry.addData("LeftForward", LeftForward.getPower());
                telemetry.addData("RightForward", RightForward.getPower());
                telemetry.addData("LeftBack", LeftBack.getPower());
                telemetry.addData("RightBack", RightBack.getPower());
                telemetry.addData("RightDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }

        StopDrive();
        sleep(200);
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

    private void turnServo(autoServoStates state, int side) {

        if (side == LEFT) {
            if (state == autoServoStates.INIT) {
                LeftTurner.setPosition(0.9);
            } else if (state == autoServoStates.GRAB) {
                LeftTurner.setPosition(0.15);
            } else if (state == autoServoStates.AWAY) {
                LeftTurner.setPosition(0.6);
            }
        } else if (side == RIGHT) {
            if (state == autoServoStates.INIT) {
                RightTurner.setPosition(0.4);
            } else if (state == autoServoStates.GRAB) {
                RightTurner.setPosition(1);
            } else if (state == autoServoStates.AWAY) {
                RightTurner.setPosition(0.6);
            }
        }
    }

    private void clampServo(autoServoStates state, int side) {

        if (side == LEFT) {
            if (state == autoServoStates.INIT) {
                LeftClamper.setPosition(1);
            } else if (state == autoServoStates.GRAB) {
                LeftClamper.setPosition(1);
            } else if (state == autoServoStates.DROP) {
                LeftClamper.setPosition(0.4);
            }
        } else if (side == RIGHT) {
            if (state == autoServoStates.INIT) {
                RightClamper.setPosition(0);
            } else if (state == autoServoStates.GRAB) {
                RightClamper.setPosition(0);
            } else if (state == autoServoStates.DROP) {
                RightClamper.setPosition(0.4);
            }
        }
    }


    //detection pipeline
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

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

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
}