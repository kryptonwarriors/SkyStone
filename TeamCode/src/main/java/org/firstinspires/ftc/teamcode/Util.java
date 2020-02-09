package org.firstinspires.ftc.teamcode;

// DC MOTOR
import com.qualcomm.robotcore.hardware.DcMotor;


// NAV


// SERVO
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;

// UTIL
import com.qualcomm.robotcore.util.ElapsedTime;

// SENSORS
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Util {

    // SERVO
    private static Servo LeftFoundation = null;
    private static Servo RightFoundation = null;
    private static Servo LeftClamp = null;
    private static Servo RightClamp = null;

    // DC MOTORS
    private static DcMotor LeftForward = null;
    private static DcMotor LeftBack = null;
    private static DcMotor RightForward = null;
    private static DcMotor RightBack = null;
    private static DcMotor LinearActuator = null;
    private static DcMotor LeftCascade = null;
    private static DcMotor RightCascade = null;


    // SENSORS
    public static BNO055IMU IMU;
    private static DistanceSensor LeftDistance,RightDistance,BackDistance;
    private static TouchSensor RBBumper, RFBumper, LBBumper, LFBumper;

    // MISC VARIABLES.
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
    public enum Exit { DontExit, ExitState, NoTimeLeftExit}
    private final int EXIT_TIME_THRESHOLD = 10;
    private int BackWallDistance = 28;
    int TargetPosition;
    static private String mySkyStonePos;
    double Power;
    private ElapsedTime myRunTime;
    public Util(Servo inLeftFoundation, Servo inRightFoundation,
                Servo inLeftClamp, Servo inRightClamp,
                DcMotor inLeftForward, DcMotor inLeftBack,
                DcMotor inRightForward, DcMotor inRightBack,
                DcMotor inLinearActuator, DcMotor inLeftCascade,
                DcMotor inRightCascade, ElapsedTime runtime, String SkyStonePos,
                DistanceSensor inLeftDistance, DistanceSensor inRightDistance, DistanceSensor inBackDistance) {
        // SERVO
        LeftFoundation = inLeftFoundation;
        RightFoundation = inRightFoundation;
        LeftClamp = inLeftClamp;
        RightClamp = inRightClamp;

        // DC MOTORS
        LeftForward = inLeftForward;
        LeftBack = inLeftBack;
        RightForward = inRightForward;
        RightBack = inRightBack;
        LinearActuator = inLinearActuator;
        LeftCascade = inLeftCascade;
        RightCascade = inRightCascade;

        //SENSORS
        LeftDistance = inLeftDistance;
        RightDistance = inRightDistance;
        BackDistance = inBackDistance;

        myRunTime = runtime;
        mySkyStonePos = SkyStonePos;

    }


    public Exit CanIExitPositionToSkyStone () {

        if( myRunTime.seconds() > EXIT_TIME_THRESHOLD ) {
            return Exit.NoTimeLeftExit;
        }
        // center = 31 || left = 23 || right = 40
        if (mySkyStonePos == "Left") {
            if (LeftDistance.getDistance(DistanceUnit.INCH) <= 23){
                return Exit.ExitState;
            }

        } else if (mySkyStonePos == "Right") {

            if (LeftDistance.getDistance(DistanceUnit.INCH) >= 40){
                return Exit.ExitState;
            }

        } else if (mySkyStonePos == "Center") {
            return Exit.ExitState;
        }

        return Exit.DontExit;

    }
    public Exit CanIExitGoToSkyStone () {

        if( myRunTime.seconds() > EXIT_TIME_THRESHOLD ) {
            return Exit.NoTimeLeftExit;
        }
        if (BackDistance.getDistance(DistanceUnit.INCH) >= 15){
            return Exit.ExitState;
        }

        return Exit.DontExit;

    }
    public Exit CanIExitPickUpStone () {

        if (myRunTime.seconds() > EXIT_TIME_THRESHOLD) {
            return Exit.NoTimeLeftExit;
        }
        if (AAuto.timer.seconds() > 2) {
            return Exit.ExitState;
        }
        return Exit.DontExit;

    }
    public Exit CanIExitWait () {

        if (myRunTime.seconds() > EXIT_TIME_THRESHOLD) {
            return Exit.NoTimeLeftExit;
        }
        if (AAuto.timer.seconds() > 0.6) {
            return Exit.ExitState;
        }
        return Exit.DontExit;

    }
    public Exit CanIExitGoToFoundation () {

        if (myRunTime.seconds() > EXIT_TIME_THRESHOLD) {
            return Exit.NoTimeLeftExit;
        }
        if (RightDistance.getDistance(DistanceUnit.INCH) <= 30) {
            return Exit.ExitState;
        }

        return Exit.DontExit;

    }
    public Exit CanIExitArmDown () {

        if (myRunTime.seconds() > EXIT_TIME_THRESHOLD) {
            return Exit.NoTimeLeftExit;
        }
        if (AAuto.timer.seconds() > 2) {
            return Exit.ExitState;
        }
        return Exit.DontExit;

    }
    public Exit CanIExitRelease () {

        if (myRunTime.seconds() > EXIT_TIME_THRESHOLD) {
            return Exit.NoTimeLeftExit;
        }
        if (AAuto.timer.seconds() > 2) {
            return Exit.ExitState;
        }
        return Exit.DontExit;

    }


    private void StartMotors(int Direction,  double Power)
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
            LeftBack.setPower(-Power);
            LeftForward.setPower(-Power);
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

    private void StopDrive()
    {
        LeftBack.setPower(0.0);
        LeftForward.setPower(0.0);
        RightForward.setPower(0.0);
        RightBack.setPower(0.0);
    }

}