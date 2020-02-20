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

/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Autonomous(name= "Dritonomous", group="Sky autonomous")

//@Disabled//comment out this line before using
public class Dritonomous extends LinearOpMode {
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


    private static Servo BackTurner = null;
    private static Servo BackClamper = null;

    private static DistanceSensor LeftDistance = null;
    private static DistanceSensor RightDistance = null;
    private static DistanceSensor BackDistance = null;


    private BNO055IMU   imu;

    PIDController pidDrive;
    PIDController RightpidDrive;
    PIDController LeftpidDrive;
    double        globalAngle, correction;
    Orientation   lastAngles = new Orientation();

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
    double STRAFE_POWER = 0.8;

    public static ElapsedTime timer = new ElapsedTime();


    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled      = false;
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



        BackTurner = hardwareMap.servo.get("BackTurner");
        BackClamper = hardwareMap.servo.get("BackClamper");

        LeftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");
        RightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");
        BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");

        LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RightForward.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        BackTurner.setPosition(0);
        BackClamper.setPosition(1);

        telemetry.addData("initDone", "yay");
        waitForStart();

        runtime.reset();
        if(opModeIsActive()) {

            RightpidDrive = new PIDController(0.2, 0, 0);
            RightpidDrive.setSetpoint(0);
            RightpidDrive.setInputRange(-90, 90);
            RightpidDrive.enable();
            LeftpidDrive = new PIDController(0.2,0,0);
            LeftpidDrive.setSetpoint(0);
            LeftpidDrive.setInputRange(-90, 90);
            LeftpidDrive.enable();
            pidDrive = new PIDController(0.1,0,0);
            pidDrive.setSetpoint(0);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();

            RightpidDrive.reset();
            LeftpidDrive.reset();
            pidDrive.reset();


            DriveWithBackDistance(FORWARD, 0.3, 18);
            //Arm Down
            BackTurner.setPosition(0.5);
            BackClamper.setPosition(0.6);
            sleep(600);
            DriveWithPID(FORWARD, 0.3, 200);
            //clamp
            BackClamper.setPosition(1);
            sleep(500);
            //arm up
            BackTurner.setPosition(0);
            sleep(500);

            DriveWithPID(BACKWARD, 0.3, 400);

            sleep(500);

            DriveWithPID(RIGHT, 0.5, 2000);

            StartMotors(FORWARD, 0.4);
            while(!(RBBumper.isPressed() || LBBumper.isPressed())) {
                telemetry.addData("not touched", "not pressed");
                telemetry.update();
            }
            StopDrive();

            //Arm down
            BackTurner.setPosition(0.55);
            sleep(500);
            //unclamp
            BackClamper.setPosition(0.3);
            sleep(700);

            DriveWithPID(BACKWARD, 0.3, 100);

            //arm up
            BackClamper.setPosition(1);
            sleep(500);
            BackTurner.setPosition(0);
            sleep(500);

            DriveWithBackDistance(BACKWARD, 0.3, 6);

            DriveWithPID(LEFT, 0.4, 950);


        }

    }








    //FUNCTIONS

    private void StartMotors (int Direction,  double Power)
    {
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
            LeftBack.setPower(Power);
            LeftForward.setPower(Power);
            RightBack.setPower(Power);
        }
        else if (Direction == BACKWARD) {
            RightForward.setPower(-Power);
            LeftBack.setPower(Power);
            LeftForward.setPower(Power);
            RightBack.setPower(-Power);
        }
        else if (Direction == LEFT) {
            LeftForward.setPower(Power);
            LeftBack.setPower(-Power);
            RightForward.setPower(Power);
            RightBack.setPower(-Power);
        }
        else if (Direction == RIGHT) {
            LeftForward.setPower(-Power);
            LeftBack.setPower(Power);
            RightForward.setPower(-Power);
            RightBack.setPower(Power);
        }
        else if (Direction == RTurn) {
            THRESH = TURNTHRESH;
            LeftForward.setPower(-Power);
            LeftBack.setPower(-Power);
            RightForward.setPower(-Power);
            RightBack.setPower(-Power);
        }
        else if (Direction == LTurn) {
            THRESH = TURNTHRESH;
            LeftForward.setPower(Power);
            LeftBack.setPower(Power);
            RightForward.setPower(Power);
            RightBack.setPower(Power);
        }
    }

    private void DriveWithPID (int direction, double power, int TargetPosition)
    {
        LeftForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightForward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightForward.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double LFpower, RFpower, LBpower, RBpower;

        if (direction == FORWARD) {
            LeftForward.setTargetPosition(-TargetPosition);
            RightForward.setTargetPosition(TargetPosition);
            LeftBack.setTargetPosition(-TargetPosition);
            RightBack.setTargetPosition(TargetPosition);
            LFpower = power;
            RFpower = power;
            LBpower = power;
            RBpower = power;
        } else if (direction == BACKWARD) {
            LeftForward.setTargetPosition(TargetPosition);
            RightForward.setTargetPosition(-TargetPosition);
            LeftBack.setTargetPosition(TargetPosition);
            RightBack.setTargetPosition(-TargetPosition);
            LFpower = -power;
            RFpower = -power;
            LBpower = -power;
            RBpower = -power;
        } else if (direction == LEFT) {
            LeftForward.setTargetPosition(TargetPosition);
            RightForward.setTargetPosition(TargetPosition);
            LeftBack.setTargetPosition(-TargetPosition);
            RightBack.setTargetPosition(-TargetPosition);
            LFpower = -power;
            RFpower = power;
            LBpower = power;
            RBpower = -power;
        } else if (direction == RIGHT) {
            LeftForward.setTargetPosition(-TargetPosition);
            RightForward.setTargetPosition(-TargetPosition);
            LeftBack.setTargetPosition(TargetPosition);
            RightBack.setTargetPosition(TargetPosition);
            LFpower = power;
            RFpower = -power;
            LBpower = -power;
            RBpower = power;
        } else {
            LFpower = 0;
            RFpower = 0;
            LBpower = 0;
            RBpower = 0;
        }

        //Math.abs(LeftForward.getCurrentPosition()) < Math.abs(LeftForward.getTargetPosition())
        while(opModeIsActive() && Math.abs(LeftForward.getCurrentPosition()) < Math.abs(LeftForward.getTargetPosition())) {

            if(direction == RIGHT)
                correction = RightpidDrive.performPID(getAngle());
            if(direction == LEFT)
                correction = LeftpidDrive.performPID(getAngle());
            else
                correction = pidDrive.performPID(getAngle());
            LeftForward.setPower(LFpower - correction);
            LeftBack.setPower(LBpower - correction);
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

    private void DriveWithBackDistance (int direction, double power, double  inches) {
        if (direction == FORWARD) {
            while(opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) < inches) {
                correction = pidDrive.performPID(getAngle());
                LeftForward.setPower(power - correction);
                LeftBack.setPower(power - correction);
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
            while(opModeIsActive() && BackDistance.getDistance(DistanceUnit.INCH) > inches) {
                correction = pidDrive.performPID(getAngle());
                LeftForward.setPower(-power - correction);
                LeftBack.setPower(-power - correction);
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

    private void DriveWithRightDistance (int direction, double power, double  inches) {
        if (direction == LEFT) {
            while(opModeIsActive() && RightDistance.getDistance(DistanceUnit.INCH) < inches) {
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
                telemetry.addData("RightDistance", RightDistance.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        } else if (direction == RIGHT) {
            while(opModeIsActive() && RightDistance.getDistance(DistanceUnit.INCH) > inches) {
                correction = RightpidDrive.performPID(getAngle());
                LeftForward.setPower(power - correction);
                LeftBack.setPower(-power - correction);
                RightForward.setPower(-power - correction);
                RightBack.setPower(power - correction);

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

    private void DriveWithLeftDistance (int direction, double power, double  inches) {
        if (direction == LEFT) {
            while(opModeIsActive() && LeftDistance.getDistance(DistanceUnit.INCH) > inches) {
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
        } else if (direction == RIGHT) {
            while(opModeIsActive() && LeftDistance.getDistance(DistanceUnit.INCH) < inches) {
                correction = RightpidDrive.performPID(getAngle());
                LeftForward.setPower(power - correction);
                LeftBack.setPower(-power - correction);
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
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {

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

    private void StopDrive()
    {
        LeftBack.setPower(0.0);
        LeftForward.setPower(0.0);
        RightForward.setPower(0.0);
        RightBack.setPower(0.0);
    }

}