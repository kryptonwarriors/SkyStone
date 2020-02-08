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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "AAuto", group = "Iterative Opmode")
@Disabled
public class AAuto extends OpMode {
    private static ElapsedTime runtime = new ElapsedTime();
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

    private static BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    private static double globalAngle;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();






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
    String SkyStonePos;

    private ElapsedTime timer = new ElapsedTime();

    //All states will go in here
    public enum RobotState {
        INIT, INITIAL_STRAFE, FORWARD_TO_SKYSTONE, ARM_DOWN;
    }

    RobotState CurrentState = RobotState.INIT;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        LeftForward = hardwareMap.dcMotor.get("LeftForward");
        RightForward = hardwareMap.dcMotor.get("RightForward");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

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

    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
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

    private void Encoder_Function(int Direction, int TargetPosition, double Power)
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

        while ( ( (Math.abs(Math.abs(LeftForward.getCurrentPosition()) - Math.abs(TargetPosition)) > THRESH)) )

        {
            telemetry.addData("Direction", Direction);
            telemetry.addData("key", "moving");
            telemetry.addData("LFCurrentPosition", LeftForward.getCurrentPosition());
            telemetry.addData("LFTargetPosition", -TargetPosition);
            telemetry.addData("RFCurrentPosition", RightForward.getCurrentPosition());
            telemetry.addData("RFTargetPosition", TargetPosition);
            telemetry.addData("LBCurrentPosition", LeftBack.getCurrentPosition());
            telemetry.addData("LBTargetPosition", TargetPosition);
            telemetry.addData("RBCurrentPosition", RightBack.getCurrentPosition());
            telemetry.addData("RBTargetPosition", -TargetPosition);
            telemetry.update();
        }

        LeftBack.setPower(0.0);
        LeftForward.setPower(0.0);
        RightForward.setPower(0.0);
        RightBack.setPower(0.0);
        telemetry.addData("Zero", "Motors stopped");
        telemetry.update();
    } // End of function



    private void StopDrive()
    {
        LeftBack.setPower(0.0);
        LeftForward.setPower(0.0);
        RightForward.setPower(0.0);
        RightBack.setPower(0.0);
    }

    private void moveUntilBackBumper (double Power) {
        RightForward.setPower(-Power);
        LeftBack.setPower(Power);
        LeftForward.setPower(Power);
        RightBack.setPower(-Power);
        while (! (LBBumper.isPressed() || RBBumper.isPressed())  ) {
            telemetry.addData("LeftBackBumper", LBBumper.isPressed());
            telemetry.addData("RightBackBumper", RBBumper.isPressed());
            telemetry.update();
        }
        StopDrive();
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (CurrentState) {

            case INIT:
                if (SkyStonePos.equals("Left")) {
                    StartMotors(LEFT, 0.4);
                }
                else if (SkyStonePos.equals("Right")) {
                    StartMotors(RIGHT, 0.4);
                }
                timer.reset();
                CurrentState = RobotState.INITIAL_STRAFE;
                break;
            case INITIAL_STRAFE:
                if (SkyStonePos.equals("Left")) {
                    if (LeftDistance.getDistance(DistanceUnit.INCH) <= 25 || timer.milliseconds() > 1) {
                        StopDrive();
                        StartMotors(FORWARD, 0.4);
                        CurrentState = RobotState.FORWARD_TO_SKYSTONE;
                    }
                    else {
                        telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                }
                else if (SkyStonePos.equals("Right")) {
                    if (LeftDistance.getDistance(DistanceUnit.INCH) >= 38.5 || timer.milliseconds() > 1300) {
                        StopDrive();
                        StartMotors(FORWARD, 0.4);
                        CurrentState = RobotState.FORWARD_TO_SKYSTONE;
                    }
                    else {
                        telemetry.addData("LeftDistance", LeftDistance.getDistance(DistanceUnit.INCH));
                        telemetry.update();
                    }
                }
                break;
            case FORWARD_TO_SKYSTONE:
                if (BackDistance.getDistance(DistanceUnit.INCH) >= 30) {
                    StopDrive();
                    timer.reset();
                    CurrentState = RobotState.ARM_DOWN;
                }
                else {
                    telemetry.addData("BackDistance", BackDistance.getDistance(DistanceUnit.INCH));
                    telemetry.update();
                }
                break;
            case ARM_DOWN:
                if(timer.milliseconds() >= 5000) {
                    StopDrive();
                }




        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
