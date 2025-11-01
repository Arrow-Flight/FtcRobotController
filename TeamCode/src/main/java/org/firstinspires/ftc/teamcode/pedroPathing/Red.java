package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class Red extends OpMode {

    // Add Actuators
    private DcMotor shooterLeft;
    private DcMotor shooterRight;
    private DcMotor Intake;
    private CRServo finger;
    private Timer MyTimer;
    private Boolean Timed;

    //
    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Define positions
    private final Pose startPose = new Pose(34.000, 135.500, Math.toRadians(-90));
    private final Pose shootpose = new Pose(20, 90.500, Math.toRadians(-135));
    private final Pose endPose = new Pose(34, 80, Math.toRadians(-90));
    private Path Path1;
    private Path Leave;

    @Override
    public void init() {
        // Set up left shooter motor
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setPower(0);
        // Set up right shooter motor
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setPower(0);
        // Set up Intake motor
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Intake.setPower(0);
        //Servo
        finger = hardwareMap.get(CRServo.class, "finger");
        finger.setDirection(DcMotorSimple.Direction.FORWARD);

        MyTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build a simple path between the two poses
        Path1 = new Path(new BezierLine(startPose, shootpose));
        Path1.setLinearHeadingInterpolation(startPose.getHeading(), shootpose.getHeading());

        Leave = new Path(new BezierLine(shootpose, endPose));
        Leave.setLinearHeadingInterpolation(shootpose.getHeading(), endPose.getHeading());

        pathTimer = new Timer();
        pathState = 0;

        telemetry.addLine("Initialized and ready");
        telemetry.update();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        follower.followPath(Path1);
        Timed = Boolean.FALSE;
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.addData("Key", MyTimer.getElapsedTimeSeconds());
        if (!follower.isBusy()) {
            if (pathState == 0) {
                if (!Timed) {
                    MyTimer.resetTimer();
                    Timed = Boolean.TRUE;
                }
                shooterLeft.setPower(1);
                shooterRight.setPower(1);
                if (MyTimer.getElapsedTimeSeconds() >= 2) {
                    pathState = 1;
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    finger.setPower(0);
                } else if (MyTimer.getElapsedTimeSeconds() >= 0.5) {
                    finger.setPower(1);
                } else {
                    finger.setPower(0);
                }
            } else if (pathState == 1) {
                follower.followPath(Leave);
                pathState = 2;
            } else if (pathState == 2) {

            }
        }
    }
}