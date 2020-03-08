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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "Red_Foundation_Park", group = "Iterative Opmode")
@Disabled

public class Red_Foundation_Park extends OpMode {
    // Declare OpMode members.
    private static ElapsedTime runtime = new ElapsedTime ();
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

    private static BNO055IMU imu;
    Orientation lastAngles = new Orientation ();
    private static double globalAngle;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters ();


    private ColorSensor Color;
    //private DistanceSensor BackDistance;
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

    private ElapsedTime timer = new ElapsedTime ();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        LeftForward = hardwareMap.dcMotor.get ( "LeftForward" );
        RightForward = hardwareMap.dcMotor.get ( "RightForward" );
        LeftBack = hardwareMap.dcMotor.get ( "LeftBack" );
        RightBack = hardwareMap.dcMotor.get ( "RightBack" );

        imu = hardwareMap.get ( BNO055IMU.class, "imu" );

        imu.initialize ( parameters );

        LinearActuator = hardwareMap.dcMotor.get ( "LinearActuator" );
        LeftCascade = hardwareMap.dcMotor.get ( "LeftCascade" );
        RightCascade = hardwareMap.dcMotor.get ( "RightCascade" );

        //BackDistance = hardwareMap.get(DistanceSensor.class, "BackDistance");
        LFBumper = hardwareMap.get ( RevTouchSensor.class, "LFBumper" );
        RFBumper = hardwareMap.get ( RevTouchSensor.class, "RFBumper" );
        LBBumper = hardwareMap.get ( RevTouchSensor.class, "LBBumper" );
        RBBumper = hardwareMap.get ( RevTouchSensor.class, "RBBumper" );


        LeftFoundation = hardwareMap.servo.get ( "LeftFoundation" );
        RightFoundation = hardwareMap.servo.get ( "RightFoundation" );
        LeftClamp = hardwareMap.servo.get ( "LeftClamp" );
        RightClamp = hardwareMap.servo.get ( "RightClamp" );

        LeftDistance = hardwareMap.get ( DistanceSensor.class, "LeftDistance" );
        RightDistance = hardwareMap.get ( DistanceSensor.class, "RightDistance" );

        //Reset Foundation Servo Positions
        LeftFoundation.setPosition ( 0.80 );
        RightFoundation.setPosition ( 0.22 );

        telemetry.addData ( "Status", "Initialized" );
        telemetry.update ();
    }

    RobotState CurrentState = RobotState.INIT;

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData ( "Left Distance", LeftDistance.getDistance ( DistanceUnit.INCH ) );
        telemetry.addData ( "Right Distance", RightDistance.getDistance ( DistanceUnit.INCH ) );
        telemetry.update ();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset ();

    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation ( AxesReference.INTRINSIC, AxesOrder.ZYX,
                                                         AngleUnit.DEGREES );

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
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

    private void Encoder_Function(int Direction, int TargetPosition, double Power) {

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

        while (((Math.abs ( Math.abs ( LeftForward.getCurrentPosition () ) - Math.abs (
                TargetPosition ) ) > THRESH))) {
            telemetry.addData ( "Direction", Direction );
            telemetry.addData ( "key", "moving" );
            telemetry.addData ( "LFCurrentPosition", LeftForward.getCurrentPosition () );
            telemetry.addData ( "LFTargetPosition", -TargetPosition );
            telemetry.addData ( "RFCurrentPosition", RightForward.getCurrentPosition () );
            telemetry.addData ( "RFTargetPosition", TargetPosition );
            telemetry.addData ( "LBCurrentPosition", LeftBack.getCurrentPosition () );
            telemetry.addData ( "LBTargetPosition", TargetPosition );
            telemetry.addData ( "RBCurrentPosition", RightBack.getCurrentPosition () );
            telemetry.addData ( "RBTargetPosition", -TargetPosition );
            telemetry.update ();
        }

        LeftBack.setPower ( 0.0 );
        LeftForward.setPower ( 0.0 );
        RightForward.setPower ( 0.0 );
        RightBack.setPower ( 0.0 );
        telemetry.addData ( "Zero", "Motors stopped" );
        telemetry.update ();
    } // End of function

    private void StopDrive() {
        LeftBack.setPower ( 0.0 );
        LeftForward.setPower ( 0.0 );
        RightForward.setPower ( 0.0 );
        RightBack.setPower ( 0.0 );
    }

    private void moveUntilBackBumper(double Power) {
        RightForward.setPower ( -Power );
        LeftBack.setPower ( Power );
        LeftForward.setPower ( Power );
        RightBack.setPower ( -Power );
        while (!(LBBumper.isPressed () || RBBumper.isPressed ())) {
            telemetry.addData ( "LeftBackBumper", LBBumper.isPressed () );
            telemetry.addData ( "RightBackBumper", RBBumper.isPressed () );
            telemetry.update ();
        }
        StopDrive ();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation ( AxesReference.INTRINSIC, AxesOrder.ZYX,
                                                 AngleUnit.DEGREES );

        globalAngle = 0;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle ();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        switch (CurrentState) {

            case INIT:
                StartMotors ( RIGHT, 0.6 );
                CurrentState = RobotState.GO_RIGHT_TO_FOUNDATION;
                break;
            case GO_RIGHT_TO_FOUNDATION:
                if (((Math.abs ( Math.abs ( LeftForward.getCurrentPosition () ) - Math.abs (
                        350 ) ) <= THRESH))) {
                    telemetry.addData ( "LeftForward Encoder", LeftForward.getCurrentPosition () );
                    telemetry.update ();
                    StopDrive ();
                    StartMotors ( FORWARD, 0.25 );
                    CurrentState = RobotState.GO_TO_FOUNDATION;
                } else {
                    telemetry.addData ( "LeftForward Encoder", LeftForward.getCurrentPosition () );
                    telemetry.addData ( "State", "Going Right to Foundation" );
                    telemetry.update ();
                }
                break;
            case GO_TO_FOUNDATION:
                if (LFBumper.isPressed () || RFBumper.isPressed ()) { //Exit Criteria
                    StopDrive ();
                    LeftFoundation.setPosition ( 0.31 );
                    RightFoundation.setPosition ( 0.73 );
                    timer.reset ();
                    CurrentState = RobotState.FOUNDATION_ARM_DOWN;
                } else {
                    telemetry.addData ( "State", "Going to Foundation" );
                    telemetry.update ();
                }
                break;
            case FOUNDATION_ARM_DOWN:
                if (timer.milliseconds () > 500) {
                    CurrentState = RobotState.PULL_FOUNDATION;
                } else {
                    telemetry.addData ( "State", "FOUNDATION_ARM_DOWN" );
                    telemetry.update ();
                }
                break;

            case PULL_FOUNDATION:
                StartMotors ( BACKWARD, 0.4 );
                CurrentState = RobotState.FOUNDATION_STOP;
                telemetry.addData ( "State", "Pulling Foundation" );
                telemetry.update ();
                break;
            case FOUNDATION_STOP:
                if (LBBumper.isPressed () || RBBumper.isPressed ()) {
                    StopDrive ();
                    LeftFoundation.setPosition ( 0.80 );
                    RightFoundation.setPosition ( 0.22 );
                    StartMotors ( LEFT, 0.4 );
                    CurrentState = RobotState.GET_CLOSE_TO_PARK;
                } else {
                    telemetry.addData ( "State", "FOUNDATION_STOP" );
                    telemetry.update ();
                }
                break;
            case GET_CLOSE_TO_PARK:
                if (((Math.abs ( Math.abs ( LeftForward.getCurrentPosition () ) - Math.abs (
                        800 ) ) <= THRESH))) {
                    StopDrive ();
                    if (LeftDistance.getDistance ( DistanceUnit.INCH ) >= 40) {
                        StartMotors ( LEFT, 0.4 );
                        CurrentState = RobotState.PARK_DOWN;
                    } else {
                        StartMotors ( LEFT, 0.4 );
                        CurrentState = RobotState.MOVE_LEFT;
                    }
                } else {
                    telemetry.addData ( "State", "GET_CLOSE_TO_PARK" );
                    telemetry.addData ( "LeftDistanceSensor",
                                        LeftDistance.getDistance ( DistanceUnit.INCH ) );
                    telemetry.update ();
                }
            case MOVE_LEFT:

                if (LeftDistance.getDistance ( DistanceUnit.INCH ) < 7) {
                    telemetry.addData ( "LeftForward Encoder", LeftForward.getCurrentPosition () );
                    telemetry.update ();
                    StopDrive ();
                    StartMotors ( FORWARD, 0.4 );
                    CurrentState = RobotState.MOVE_UP;
                } else {
                    telemetry.addData ( "LeftForward Encoder", LeftForward.getCurrentPosition () );
                    telemetry.addData ( "State", "MOVE_LEFT" );
                    telemetry.update ();
                }
                break;
            case MOVE_UP:
                if (((Math.abs ( Math.abs ( LeftForward.getCurrentPosition () ) - Math.abs (
                        300 ) ) <= THRESH))) {
                    telemetry.addData ( "LeftForward Encoder", LeftForward.getCurrentPosition () );
                    telemetry.update ();
                    StopDrive ();
                    StartMotors ( LEFT, 0.4 );
                    CurrentState = RobotState.MOVE_LEFT_TO_SKYBRIDGE;
                } else {
                    telemetry.addData ( "LeftForward Encoder", LeftForward.getCurrentPosition () );
                    telemetry.addData ( "State", "MOVE_UP" );
                    telemetry.update ();
                }
                break;
            case MOVE_LEFT_TO_SKYBRIDGE:
                if (((Math.abs ( Math.abs ( LeftForward.getCurrentPosition () ) - Math.abs (
                        400 ) ) <= THRESH))) {
                    telemetry.addData ( "LeftForward Encoder", LeftForward.getCurrentPosition () );
                    telemetry.update ();
                    StopDrive ();
                    CurrentState = RobotState.STOP;
                } else {
                    telemetry.addData ( "LeftForward Encoder", LeftForward.getCurrentPosition () );
                    telemetry.addData ( "State", "MOVE_LEFT_TO_SKYBRIDGE" );
                    telemetry.update ();
                }
                break;
            case PARK_DOWN:
                if (((Math.abs ( Math.abs ( LeftForward.getCurrentPosition () ) - Math.abs (
                        900 ) ) <= THRESH))) {
                    telemetry.addData ( "LeftForward Encoder", LeftForward.getCurrentPosition () );
                    telemetry.update ();
                    StopDrive ();
                    CurrentState = RobotState.STOP;
                } else {
                    telemetry.addData ( "LeftDistanceSensor",
                                        LeftDistance.getDistance ( DistanceUnit.INCH ) );
                    telemetry.addData ( "LeftForward Encoder", LeftForward.getCurrentPosition () );
                    telemetry.addData ( "State", "PARK DOWN" );
                    telemetry.update ();
                }
                break;
            case STOP:
                LeftForward.setPower ( 0 );
                LeftBack.setPower ( 0 );
                RightForward.setPower ( 0 );
                RightBack.setPower ( 0 );
                break;


        }
    }

    //All states will go in here
    public enum RobotState {
        INIT, GO_TO_FOUNDATION, GO_RIGHT_TO_FOUNDATION, FOUNDATION_ARM_DOWN,
        PULL_FOUNDATION, FOUNDATION_STOP, PARKING, MOVE_UP, MOVE_LEFT, MOVE_LEFT_TO_SKYBRIDGE,
        PARK_DOWN, GET_CLOSE_TO_PARK, STOP
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

}