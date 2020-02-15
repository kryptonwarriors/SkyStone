/**
 * package org.firstinspires.ftc.teamcode;
 * <p>
 * import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 * import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 * <p>
 * import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
 * import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
 * import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
 * import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
 * import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
 * <p>
 * import com.acmerobotics.dashboard.FtcDashboard;
 * import com.acmerobotics.dashboard.canvas.Canvas;
 * import com.acmerobotics.dashboard.config.Config;
 * import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
 * import com.acmerobotics.roadrunner.control.PIDCoefficients;
 * import com.acmerobotics.roadrunner.control.PIDFController;
 * import com.acmerobotics.roadrunner.drive.DriveSignal;
 * import com.acmerobotics.roadrunner.drive.MecanumDrive;
 * import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
 * import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
 * import com.acmerobotics.roadrunner.geometry.Pose2d;
 * import com.acmerobotics.roadrunner.profile.MotionProfile;
 * import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
 * import com.acmerobotics.roadrunner.profile.MotionState;
 * import com.acmerobotics.roadrunner.trajectory.Trajectory;
 * import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
 * import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
 * import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
 * import com.acmerobotics.roadrunner.util.NanoClock;
 * import com.qualcomm.robotcore.hardware.DcMotor;
 * import org.firstinspires.ftc.teamcode.util.DashboardUtil;
 * <p>
 * import java.util.ArrayList;
 * import java.util.List;
 * import com.qualcomm.robotcore.hardware.DcMotor;
 * import com.qualcomm.robotcore.hardware.DcMotorSimple;
 * import com.qualcomm.robotcore.hardware.Servo;
 * import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
 * import com.qualcomm.robotcore.util.ElapsedTime;
 *
 * @Autonomous(name = "RoadKill", group = "")
 * public class RoadKill extends LinearOpMode {
 * <p>
 * private DcMotor RightForward, RightBack, LeftForward, LeftBack, RightCascade, LeftCascade, LinearActuator;
 * public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(0, 0, 0);
 * public static PIDCoefficients HEADING_PID = new PIDCoefficients(0, 0, 0);
 * private ElapsedTime runtime = new ElapsedTime();
 * @Override public void runOpMode() {
 * <p>
 * telemetry.update();
 * waitForStart();
 * if (opModeIsActive()) {
 * while (opModeIsActive()) {
 * <p>
 * }
 * }
 * }
 * }
 **/