package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Blue.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.*;

import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.pedroPathing.pedro.Constants;

@Autonomous(preselectTeleOp="MainOpBlue")
public class BlueShort extends OpMode {

    @Override
    public void init() {
        // Set Up servoCamera
        servoCamera = hardwareMap.get(Servo.class, "servoCamera");
        servoCamera.scaleRange(0.3, 1.0);
        servoCamera.setPosition(0.4);

        //Set Up intake
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //Set Up upper
        upper = hardwareMap.get(DcMotor.class, "upper");
        upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Up shooter motors
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        pidfController = new PIDFController(new com.pedropathing.control.PIDFCoefficients(0.002, 0, 0, 0.76));
        pidfController.setTargetPosition(2600);


        // Set Up Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline); // use your AprilTag pipeline

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        pathState = 0;

        // Add Timers
        pathTimer = new Timer();
        timeout = new Timer();
    }

    @Override
    public void start() {
        limelight.start();
        pathTimer.resetTimer();
        timeout.resetTimer();
    }

    @Override
    public void loop() {
        vel = -getVelocity();
        telemetry.addData("vel", vel);
        telemetry.addData("shots", shots);
        telemetry.update();
        follower.update();

        // Step 1: Initial Move
        if (pathState == 0 && !follower.isBusy()) {
            currentPose = follower.getPose();

            toShoot = new Path(new BezierLine(currentPose, preShoot));
            toShoot.setLinearHeadingInterpolation(currentPose.getHeading(), preShoot.getHeading());

            follower.followPath(toShoot);

            pathTimer.resetTimer();
            pathState = 1;
        }

        // Step 2: Calculate Offsets and Fix Position
        else if (pathState ==1 && pathTimer.getElapsedTimeSeconds() > 2 && !follower.isBusy()) {
            currentPose = follower.getPose();
            xError = getStartingError().getX();
            yError = getStartingError().getY();

            Pose correctedTarget = new Pose(shootPose.getX() + xError, shootPose.getY() + yError, shootPose.getHeading());

            toShoot = new Path(new BezierLine(currentPose, correctedTarget));
            toShoot.setLinearHeadingInterpolation(currentPose.getHeading(), correctedTarget.getHeading());
            follower.followPath(toShoot);
            pathTimer.resetTimer();
            shots = 0;
            spunUp = false;
            shootState = 1;
            pathState = 2;
        }

        else if (pathState == 2 && !follower.isBusy()) {
            currentPose = getCorrectedPose();
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
            intake.setPower(0);
            upper.setPower(0);

            goToEnd = new Path(new BezierLine(currentPose, endPose));
            goToEnd.setLinearHeadingInterpolation(currentPose.getHeading(),endPose.getHeading());

            follower.followPath(goToEnd);
            pathState = 3;
        }
    }
}