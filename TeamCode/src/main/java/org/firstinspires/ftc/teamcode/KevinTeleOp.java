package org.firstinspires.ftc.teamcode;
// import Rahuls's Genius & IQ
// import GOAT || AMAN
//import com.qualcomm.robotcore.brain.Moni;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "KevinTeleOp", group = "")
public class KevinTeleOp extends LinearOpMode {

  private DcMotor RightForward, RightBack, LeftForward, LeftBack;
  private DcMotor LeftCascade, RightCascade;
  private DcMotor LinearActuator;
  private Servo LeftClamp, RightClamp, RightTurner, LeftTurner, RightClamper, LeftClamper;
  private ElapsedTime runtime = new ElapsedTime();
  private double Multiplier = 0.7;
  private double StrafeMultiplier = -0.8;

 private double Scale (double Input) {
      double Output = Input * Math.abs(Input);
      return Output;
 }

  @Override
  public void runOpMode() {

    RightForward = hardwareMap.dcMotor.get("RightForward");
    RightBack = hardwareMap.dcMotor.get("RightBack");
    LeftForward = hardwareMap.dcMotor.get("LeftForward");
    LeftBack = hardwareMap.dcMotor.get("LeftBack");
    LeftClamp = hardwareMap.servo.get("LeftClamp");
    RightClamp = hardwareMap.servo.get("RightClamp");
    RightTurner = hardwareMap.servo. get("RightTurner");
    RightClamper = hardwareMap.servo.get("RightClamper");
    LeftTurner = hardwareMap.servo. get("LeftTurner");
    LeftClamper = hardwareMap.servo.get("LeftClamper");
    RightCascade = hardwareMap.dcMotor.get("RightCascade");
    LeftCascade = hardwareMap.dcMotor.get("LeftCascade");
    LinearActuator = hardwareMap.dcMotor.get("LinearActuator");


    RightForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftForward.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    RightForward.setDirection(DcMotorSimple.Direction.REVERSE);
    LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    
    
    RightForward.setPower(0);
    RightBack.setPower(0);
    LeftForward.setPower(0);
    LeftBack.setPower(0);
    telemetry.addData(">", "INIT DONE");
// RESET TIME
    runtime.reset();

    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
       if(Math.abs(gamepad1.right_trigger) > 0.01){
         // Strafing to the Right
        LeftForward.setPower(StrafeMultiplier * Scale(gamepad1.right_trigger));
        LeftBack.setPower(-StrafeMultiplier * Scale(gamepad1.right_trigger));
        RightForward.setPower(-StrafeMultiplier * Scale(gamepad1.right_trigger));
        RightBack.setPower(StrafeMultiplier * Scale(gamepad1.right_trigger));
       } else if(Math.abs(gamepad1.left_trigger) > 0.01){
        // Strafing to the Left
        LeftForward.setPower(-StrafeMultiplier * Scale(gamepad1.left_trigger));
        LeftBack.setPower(StrafeMultiplier * Scale(gamepad1.left_trigger));
        RightForward.setPower(StrafeMultiplier * Scale(gamepad1.left_trigger));
        RightBack.setPower(-StrafeMultiplier * Scale(gamepad1.left_trigger));
      } else if (gamepad1.y) {
        RightBack.setPower(-0.4);
        RightForward.setPower(-0.4);
        LeftForward.setPower(-0.4);
        LeftBack.setPower(-0.4);
      } else if (gamepad1.a) {
        RightBack.setPower(0.4);
        RightForward.setPower(0.4);
        LeftForward.setPower(0.4);
        LeftBack.setPower(0.4);
      } else if (gamepad1.right_bumper) {
        RightBack.setPower(-0.8);
        RightForward.setPower(-0.8);
        LeftForward.setPower(0.8);
        LeftBack.setPower(0.8);
      } else if (gamepad1.left_bumper) {
        RightBack.setPower(0.8);
        RightForward.setPower(0.8);
        LeftForward.setPower(-0.8);
        LeftBack.setPower(-0.8);
      } else {
        RightBack.setPower(Multiplier * Scale(gamepad1.right_stick_y));
        RightForward.setPower(Multiplier * Scale(gamepad1.right_stick_y));
        LeftForward.setPower(Multiplier * Scale(gamepad1.left_stick_y));
        LeftBack.setPower(Multiplier * Scale(gamepad1.left_stick_y));
      }
      if (gamepad2.a == true) {
        LeftClamp.setPosition(0.9);
        RightClamp.setPosition(0.3);
          // Clamp in & CLOSE
      }
      if (gamepad2.b == true) {
        LeftClamp.setPosition(0.8);
        RightClamp.setPosition(0.5);
          // Clamp out & OPEN
      }
      /*if (gamepad2.x == true) {
          LeftFoundation.setPosition(0);
          RightFoundation.setPosition(0.5);
          // Down
        }
        if (gamepad2.y == true) {
          LeftFoundation.setPosition(0.6);
          RightFoundation.setPosition(0.91);
          // Up[
          ]
        }*/
        if (gamepad2.left_bumper == true) {
          /*LeftCascade.setPower(-0.2);
          RightCascade.setPower(0.2);
          sleep(100);
          LeftCascade.setPower(0);
          RightCascade.setPower(0);*/
          //Grab Stone
          //BackTurner.setPosition(0.5);
          //Down to Grab Stone
          //BackClamper.setPosition(1);
        }
        if (gamepad2.right_bumper == true) {
          //Up
          //BackClamper.setPosition(0.6);
        }
        if (gamepad2.dpad_up) {
          //BackTurner.setPosition(0);
        }
        
        LinearActuator.setPower(gamepad2.right_stick_y * -0.75);
        RightCascade.setPower(gamepad2.left_stick_y * -0.9);
        LeftCascade.setPower(gamepad2.left_stick_y * 0.9);
        

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("RightForward", RightForward.getPower());
        telemetry.addData("LeftForward", LeftForward.getPower());
        telemetry.addData("RightBack", RightBack.getPower());
        telemetry.addData("LeftBack", LeftBack.getPower());
        telemetry.addData("LeftTrigger", gamepad1.left_trigger);
        telemetry.addData("RightTrigger", gamepad1.right_trigger);
        telemetry.addData("LeftCascade", LeftCascade.getPower());
        telemetry.addData("RightCascade", RightCascade.getPower());
        telemetry.update();
      }
    }
  }
}

