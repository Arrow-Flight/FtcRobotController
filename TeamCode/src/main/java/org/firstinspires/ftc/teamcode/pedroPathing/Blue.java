package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.Blue.*;
import static org.firstinspires.ftc.teamcode.pedroPathing.AutoConstants.*;

import com.pedropathing.geometry.*;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.teamcode.pedroPathing.pedro.Constants;

@Autonomous
public class Blue extends OpMode {

    @Override
    public void init() {
        // Set Up servoCamera
        servoCamera = hardwareMap.get(Servo.class, "servoCamera");
        servoCamera.scaleRange(0.3, 1.0);
        servoCamera.setPosition(0.3);

        //Set Up intake
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //Set Up upper
        upper = hardwareMap.get(DcMotor.class, "upper");
        upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set Up shooter motors
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(42.0, 0, 0, 18.5);
        shooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Set Up Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline); // use your AprilTag pipeline

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        pathState = 0;

        // Build Paths
        shootToFirstSpike = new Path(new BezierLine(shootPose, firstSpikeInitial));
        shootToFirstSpike.setLinearHeadingInterpolation(shootPose.getHeading(),firstSpikeInitial.getHeading());

        firstSpike = new Path(new BezierLine(firstSpikeInitial, firstSpikeFinal));
        firstSpike.setLinearHeadingInterpolation(firstSpikeInitial.getHeading(),firstSpikeFinal.getHeading());

        shootFromFirstSpike = new Path(new BezierLine(firstSpikeInitial, shootPose));
        shootFromFirstSpike.setLinearHeadingInterpolation(firstSpikeFinal.getHeading(),shootPose.getHeading());

        shootToSecondSpike = new Path(new BezierLine(shootPose,secondSpikeInitial));
        shootToSecondSpike.setLinearHeadingInterpolation(shootPose.getHeading(),secondSpikeInitial.getHeading());

        secondSpike = new Path(new BezierLine(secondSpikeInitial,secondSpikeFinal));
        secondSpike.setLinearHeadingInterpolation(secondSpikeInitial.getHeading(),secondSpikeFinal.getHeading());

        shootFromSecondSpike = new Path(new BezierLine(secondSpikeFinal, shootPose));
        shootFromSecondSpike.setLinearHeadingInterpolation(secondSpikeFinal.getHeading(), shootPose.getHeading());

        shootToThirdSpike = new Path(new BezierLine(shootPose,thirdSpikeInitial));
        shootToThirdSpike.setLinearHeadingInterpolation(shootPose.getHeading(),thirdSpikeInitial.getHeading());

        thirdSpike = new Path(new BezierLine(thirdSpikeInitial,thirdSpikeFinal));
        thirdSpike.setLinearHeadingInterpolation(thirdSpikeInitial.getHeading(),thirdSpikeFinal.getHeading());

        shootFromThirdSpike = new Path(new BezierLine(thirdSpikeFinal, shootPose));
        shootFromThirdSpike.setLinearHeadingInterpolation(thirdSpikeFinal.getHeading(), shootPose.getHeading());

        goToEnd = new Path(new BezierLine(shootPose, endPose));
        goToEnd.setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading());

        // Add Timer
        pathTimer = new Timer();

        // Initial Telemetry
        telemetry.addLine("Initialized and ready");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        vel = (shooterRight.getVelocity() + shooterLeft.getVelocity())/2;
        follower.update();
        telemetry.addData("Step:", pathState);
        telemetry.update();


        // Step 1: Shoot Initial Balls
        if (pathState == 0 && subState == 0 && !follower.isBusy()) {
            subState = 1;
        }
        if (pathState == 0) {
            Shoot(1, 3, shootPose, telemetry);
        }

        // Step 2: Move to Get Balls From First Spike
        else if (pathState == 1 && !follower.isBusy()) {
           follower.followPath(shootToFirstSpike);
           pathState = 2;
        }

        // Step 3: Intake Balls On First Spike
        else if (pathState == 2 && !follower.isBusy()) {
            intake.setPower(1);
            follower.followPath(firstSpike);
            pathState = 3;
        }

        // Step 4: Shoot First Spike Ball
        else if (pathState == 3 && subState == 0 && !follower.isBusy()) {
            intake.setPower(0);
            subState = 1;
        }
        if (pathState == 3) {
            Shoot(4, 3, shootPose, telemetry);
        }

        // Step 5: Move to Get Balls From Second Spike
        else if (pathState == 4 && !follower.isBusy()) {
            follower.followPath(shootToSecondSpike);
            pathState = 5;
        }

        // Step 6: Intake Balls On Second Spike
        else if (pathState == 5 && !follower.isBusy()) {
            intake.setPower(1);
            follower.followPath(secondSpike);
            pathState = 6;
        }

        // Step 7: Shoot Second Spike Balls
        else if (pathState == 6 && subState == 0 && !follower.isBusy()) {
            intake.setPower(0);
            subState = 1;
        }
        if (pathState == 6) {
            Shoot(7, 3, shootPose, telemetry);
        }

        // Step 8:  Move to Get Balls From Third Spike
        else if (pathState == 7 && !follower.isBusy()) {
            follower.followPath(shootToThirdSpike);
            pathState = 8;
        }

        // Step 9: Intake Balls On Third Spike
        else if (pathState == 8 && !follower.isBusy()) {
            intake.setPower(1);
            follower.followPath(thirdSpike);
            pathState = 9;
        }

        // Step 10: Shoot Third Spike Balls
        else if (pathState == 9 && subState == 0 && !follower.isBusy()) {
            intake.setPower(0);
            subState = 1;
        }
        if (pathState == 9) {
            Shoot(10, 3, shootPose, telemetry);
        }

        // Step 11: Go To End
        else if (pathState == 10 && !follower.isBusy()) {
            follower.followPath(goToEnd);
            pathState = 11;
        }
    }
}