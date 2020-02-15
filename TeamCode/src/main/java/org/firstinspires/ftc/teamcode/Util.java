package org.firstinspires.ftc.teamcode;

// DC MOTOR

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// NAV
// SERVO
// UTIL
// SENSORS


public class Util {

    /* MISC VARIABLES */
    private int FORWARD = 0;
    // SERVO
    private Servo LeftFoundation = null;
    private Servo RightFoundation = null;
    private Servo LeftClamp = null;
    private Servo RightClamp = null;
    // DC MOTORS
    private DcMotor LeftForward = null;
    private DcMotor LeftBack = null;
    private DcMotor RightForward = null;
    private DcMotor RightBack = null;
    private DcMotor LinearActuator = null;
    private DcMotor LeftCascade = null;
    private DcMotor RightCascade = null;
    // SENSORS
    private BNO055IMU IMU;
    private DistanceSensor LeftDistance, RightDistance, BackDistance;
    private TouchSensor RBBumper, RFBumper, LBBumper, LFBumper;
    private int BACKWARD = 1;
    private int LEFT = 2;
    private int RIGHT = 3;
    int UP = 4;
    int WALL = 5;
    private int RTurn = 6;
    private int LTurn = 7;
    int EXTEND = 8;
    int RETRACT = 9;
    private int THRESH = 15;
    private int ALL_THRESH = 15;
    private int TURNTHRESH = 30;

    Util(ElapsedTime runtime, String SkyStonePos, HardwareMap hardwareMap) {

        /* SERVO */
        RightClamp = hardwareMap.get ( Servo.class, "RightClamp" );
        LeftClamp = hardwareMap.get ( Servo.class, "LeftClamp" );
        LeftFoundation = hardwareMap.get ( Servo.class, "LeftFoundation" );
        RightFoundation = hardwareMap.get ( Servo.class, "RightFoundation" );

        /* DC MOTORS */
        LeftForward = hardwareMap.get ( DcMotor.class, "LeftForward" );
        RightForward = hardwareMap.get ( DcMotor.class, "RightForward" );
        LeftBack = hardwareMap.get ( DcMotor.class, "LeftBack" );
        RightBack = hardwareMap.get ( DcMotor.class, "RightBack" );
        LinearActuator = hardwareMap.get ( DcMotor.class, "LinearActuator" );
        LeftCascade = hardwareMap.get ( DcMotor.class, "LeftCascade" );
        RightCascade = hardwareMap.get ( DcMotor.class, "RightCascade" );

        /* SENSORS */
        IMU = hardwareMap.get ( BNO055IMU.class, "IMU" );
        LeftDistance = hardwareMap.get ( DistanceSensor.class, "LeftDistance" );
        RightDistance = hardwareMap.get ( DistanceSensor.class, "RightDistance" );
        BackDistance = hardwareMap.get ( DistanceSensor.class, "BackDistance" );
        RBBumper = hardwareMap.get ( TouchSensor.class, "RBBumper" );
        RFBumper = hardwareMap.get ( TouchSensor.class, "RFBumper" );
        LBBumper = hardwareMap.get ( TouchSensor.class, "LBBumper" );
        LFBumper = hardwareMap.get ( TouchSensor.class, "LFBumper" );


        LeftForward.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );
        LeftBack.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );
        RightForward.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );
        RightBack.setZeroPowerBehavior ( DcMotor.ZeroPowerBehavior.BRAKE );

        /* VARIABLES */
        myRunTime = runtime;
        mySkyStonePos = SkyStonePos;
    }

    private final int EXIT_TIME_THRESHOLD = 20;
    private int BackWallDistance = 28;
    int TargetPosition;
    static private String mySkyStonePos;
    double Power;
    private ElapsedTime myRunTime;

    public Exit CanIExitGoToSkyStone() {

        if (myRunTime.seconds () > EXIT_TIME_THRESHOLD) {
            return Exit.NoTimeLeftExit;
        }
        if (BackDistance.getDistance ( DistanceUnit.INCH ) >= 12) {
            return Exit.ExitState;
        }


        return Exit.DontExit;

    }

    public Exit CanIExitPickUpStone() {

        if (myRunTime.seconds () > EXIT_TIME_THRESHOLD) {
            return Exit.NoTimeLeftExit;
        }
        if (AAuto.stateTimer.seconds () > 2) {
            return Exit.ExitState;
        }
        return Exit.DontExit;

    }

    public Exit CanIExitWait() {

        if (myRunTime.seconds () > EXIT_TIME_THRESHOLD) {
            return Exit.NoTimeLeftExit;
        }
        if (AAuto.stateTimer.seconds () > 0.6) {
            return Exit.ExitState;
        }
        return Exit.DontExit;

    }

    public Exit CanIExitGoToFoundation() {

        if (myRunTime.seconds () > EXIT_TIME_THRESHOLD) {
            return Exit.NoTimeLeftExit;
        }
        if (RightDistance.getDistance ( DistanceUnit.INCH ) <= 30) {
            return Exit.ExitState;
        }

        return Exit.DontExit;

    }

    public Exit CanIExitArmDown() {

        if (myRunTime.seconds () > EXIT_TIME_THRESHOLD) {
            return Exit.NoTimeLeftExit;
        }
        if (AAuto.stateTimer.seconds () > 2) {
            return Exit.ExitState;
        }
        return Exit.DontExit;

    }

    public Exit CanIExitRelease() {

        if (myRunTime.seconds () > EXIT_TIME_THRESHOLD) {
            return Exit.NoTimeLeftExit;
        }
        if (AAuto.stateTimer.seconds () > 2) {
            return Exit.ExitState;
        }
        return Exit.DontExit;

    }

    private void StartMotors(int Direction, double Power) {

        LeftForward.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        RightForward.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        LeftBack.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );
        RightBack.setMode ( DcMotor.RunMode.STOP_AND_RESET_ENCODER );

        LeftForward.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        RightForward.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        LeftBack.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
        RightBack.setMode ( DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        THRESH = ALL_THRESH;
        if (Direction == FORWARD) {
            RightForward.setPower ( Power );
            LeftBack.setPower ( -Power );
            LeftForward.setPower ( -Power );
            RightBack.setPower ( Power );
        } else if (Direction == BACKWARD) {
            RightForward.setPower ( -Power );
            LeftBack.setPower ( Power );
            LeftForward.setPower ( Power );
            RightBack.setPower ( -Power );
        } else if (Direction == LEFT) {
            LeftForward.setPower ( Power );
            LeftBack.setPower ( -Power );
            RightForward.setPower ( Power );
            RightBack.setPower ( -Power );
        } else if (Direction == RIGHT) {
            LeftForward.setPower ( -Power );
            LeftBack.setPower ( Power );
            RightForward.setPower ( -Power );
            RightBack.setPower ( Power );
        } else if (Direction == RTurn) {
            THRESH = TURNTHRESH;
            LeftForward.setPower ( -Power );
            LeftBack.setPower ( -Power );
            RightForward.setPower ( -Power );
            RightBack.setPower ( -Power );
        } else if (Direction == LTurn) {
            THRESH = TURNTHRESH;
            LeftForward.setPower ( Power );
            LeftBack.setPower ( Power );
            RightForward.setPower ( Power );
            RightBack.setPower ( Power );
        }
    }

    private void StopDrive() {
        LeftBack.setPower ( 0.0 );
        LeftForward.setPower ( 0.0 );
        RightForward.setPower ( 0.0 );
        RightBack.setPower ( 0.0 );
    }

    public enum Exit {DontExit, ExitState, NoTimeLeftExit}

}